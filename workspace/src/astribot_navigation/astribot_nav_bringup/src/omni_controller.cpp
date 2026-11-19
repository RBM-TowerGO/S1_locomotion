#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_core/goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "route_collision.hpp"

namespace towergo {
using towergo_route::Pose;

static Pose planar(const geometry_msgs::msg::Pose & p) {
  return {p.position.x, p.position.y, tf2::getYaw(p.orientation)};
}
static double distance(Pose a, Pose b) {return std::hypot(a.x-b.x, a.y-b.y);}

// First fraction of a projected XY step inside the goal circle.
static double positionEntry(Pose from, Pose goal, double dx, double dy, double tolerance) {
  const double gx=goal.x-from.x, gy=goal.y-from.y;
  const double outside=gx*gx+gy*gy-tolerance*tolerance;
  if (outside<=0.) return 0.;
  const double along=gx*dx+gy*dy;
  const double discriminant=along*along-(dx*dx+dy*dy)*outside;
  if (along<=0. || discriminant<0.) return std::numeric_limits<double>::infinity();
  return outside/(along+std::sqrt(discriminant));
}

class OmniController : public nav2_core::Controller {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_, frame_;
  std::vector<Pose> path_;
  size_t next_ = 0;
  std::mutex mutex_;
  double limit_ = .30, horizon_ = 1.5;

public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    node_=parent.lock(); name_=name; tf_=tf; costmap_=costmap;
    for (auto kv : {std::make_pair("desired_linear_vel", .05),
                    std::make_pair("collision_time_horizon", 1.5)}) {
      if (!node_->has_parameter(name+"."+kv.first))
        node_->declare_parameter(name+"."+kv.first, kv.second);
    }
    horizon_=node_->get_parameter(name+".collision_time_horizon").as_double();
  }
  void cleanup() override {node_.reset(); costmap_.reset(); tf_.reset();}
  void activate() override {}
  void deactivate() override {}
  void setSpeedLimit(const double & value, const bool & percent) override {
    std::lock_guard<std::mutex> lock(mutex_);
    double desired=node_->get_parameter(name_+".desired_linear_vel").as_double();
    limit_=value==0 ? .30 : std::clamp(percent ? desired*value/100. : value, 0., .30);
  }
  void setPlan(const nav_msgs::msg::Path & plan) override {
    std::lock_guard<std::mutex> lock(mutex_);
    path_.clear(); next_=0; frame_=plan.header.frame_id;
    for (const auto & pose : plan.poses) path_.push_back(planar(pose.pose));
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist &,
    nav2_core::GoalChecker * checker) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path_.empty()) throw nav2_core::PlannerException("Empty navigation path");
    Pose p, local;
    const auto timeout=tf2::durationFromSec(.20);
    try {
      p=planar(tf_->transform(pose,frame_,timeout).pose);
      local=planar(tf_->transform(pose,costmap_->getGlobalFrameID(),timeout).pose);
    } catch (const tf2::ExtrapolationException & ex) {
      // Humble retries PlannerException with zero velocity during failure_tolerance.
      // Resolve both frames before advancing the path cursor.
      throw nav2_core::PlannerException(std::string("Waiting for time-aligned TF: ")+ex.what());
    }
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist unused;
    checker->getTolerances(tolerance, unused);
    const double xy=tolerance.position.x, angle=std::abs(tf2::getYaw(tolerance.orientation));
    // Follow position and yaw independently. Never look past a turn at fixed XY.
    while (next_+1<path_.size()) {
      auto a=path_[next_], b=path_[next_+1];
      bool turning=distance(a,b)<1e-5 && std::abs(towergo_route::angle(b.yaw-a.yaw))>.005;
      if (distance(p,a)>(turning ? xy : .10) ||
          std::abs(towergo_route::angle(a.yaw-p.yaw))>(turning ? angle : .18)) break;
      ++next_;
    }
    const auto t=path_[next_];
    const bool final=next_+1==path_.size();
    const double dx=t.x-p.x, dy=t.y-p.y;
    double vx=1.6*(std::cos(p.yaw)*dx+std::sin(p.yaw)*dy);
    double vy=1.6*(-std::sin(p.yaw)*dx+std::cos(p.yaw)*dy);
    double w=std::clamp(1.4*towergo_route::angle(t.yaw-p.yaw), -.30, .30);
    if ((final || (next_>0 && distance(path_[next_-1],t)<1e-5)) && distance(p,t)<=xy) vx=vy=0.;
    if (std::abs(towergo_route::angle(t.yaw-p.yaw))<angle*.35) w=0.;
    // Ignore sub-millimetre cross-axis residuals on straight primitives.
    const double cross_deadband=std::min(.001,xy*.5);
    if (std::abs(vx)<cross_deadband) vx=0.;
    if (std::abs(vy)<cross_deadband) vy=0.;
    const double speed=std::min(limit_, node_->get_parameter(name_+".desired_linear_vel").as_double());
    double scale=std::min(1.,speed/std::max(1e-12,std::hypot(vx,vy)));
    if (vy!=0.) scale=std::min(scale,.05/std::abs(vy));
    if (vx<0.) scale=std::min(scale,.03/-vx);
    vx*=scale; vy*=scale;

    // Project the actual body twist through the existing static footprint checker.
    auto * map=costmap_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
    const auto footprint=costmap_->getRobotFootprint();
    const double duration=std::min(horizon_, std::max(
      std::hypot(vx,vy)>0 ? distance(p,t)/std::hypot(vx,vy) : 0.,
      w!=0 ? std::abs(towergo_route::angle(t.yaw-p.yaw)/w) : 0.));
    const int steps=std::max(1,int(std::ceil(duration*std::max(std::hypot(vx,vy),std::abs(w))/.01)));
    const bool final_nudge=final && name_=="FinalNudge";
    // Express the fixed goal in the same costmap frame as the swept footprint.
    const double offset=local.yaw-p.yaw, c=std::cos(offset), s=std::sin(offset);
    const Pose goal{local.x+c*dx-s*dy, local.y+s*dx+c*dy, t.yaw+offset};
    const double dt=duration/steps;
    bool holding_xy=false, reached=false;
    auto q=local;
    for (int i=0; i<=steps; ++i) {
      if (!towergo_route::footprint_free(map,footprint,q))
        throw nav2_core::PlannerException("Collision detected ahead");
      if (reached || i==steps) break;  // includes the stopping footprint
      const double mid=q.yaw+w*dt*.5;
      double step_x=dt*(vx*std::cos(mid)-vy*std::sin(mid));
      double step_y=dt*(vx*std::sin(mid)+vy*std::cos(mid));
      double step_yaw=w*dt;
      if (final_nudge) {
        const double yaw_left=std::abs(towergo_route::angle(goal.yaw-q.yaw));
        const double xy_entry=holding_xy ? 0. : positionEntry(q,goal,step_x,step_y,xy);
        if (xy_entry<=1.) {
          holding_xy=true;  // Nudge holds position while finishing the turn.
          step_x*=xy_entry;
          step_y*=xy_entry;
          const double yaw_entry=yaw_left<=angle ? 0. :
            (step_yaw!=0. ? (yaw_left-angle)/std::abs(step_yaw) : std::numeric_limits<double>::infinity());
          const double stop=std::max(xy_entry,yaw_entry);
          if (stop<=1.) {step_yaw*=stop; reached=true;}
        }
        step_yaw=std::copysign(std::min(std::abs(step_yaw),yaw_left),step_yaw);
      }
      q.x+=step_x; q.y+=step_y; q.yaw+=step_yaw;
    }
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp=node_->now(); out.header.frame_id=costmap_->getBaseFrameID();
    out.twist.linear.x=vx; out.twist.linear.y=vy; out.twist.angular.z=w;
    return out;
  }
};
}
PLUGINLIB_EXPORT_CLASS(towergo::OmniController, nav2_core::Controller)
