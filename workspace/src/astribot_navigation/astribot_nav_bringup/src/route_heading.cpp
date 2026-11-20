#include "route_heading.hpp"
#include <nav2_mppi_controller/controller.hpp>
#include <nav2_mppi_controller/critic_function.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace towergo {
// MPPI evaluates critics synchronously within this controller call. Keep the
// objective scoped to that call, including when the optimizer throws.
static thread_local std::optional<double> yaw_offset;
struct HeadingScope {
  ~HeadingScope() {yaw_offset.reset();}
};

class RouteMPPI : public nav2_mppi_controller::MPPIController {
  RouteHeading heading_;
  std::string frame_, task_, requested_task_;
  std::vector<double> requested_goal_;
  bool active_=false;
  geometry_msgs::msg::Pose routePose(const geometry_msgs::msg::PoseStamped &pose) const {
    try {
      return tf_buffer_->transform(pose,frame_).pose;
    } catch(const tf2::ExtrapolationException &) {
      // AMCL can lag current odometry. Use a recent correction for this soft
      // objective only; MPPI validates transforms for the actual command.
      const auto transform=tf_buffer_->lookupTransform(frame_,pose.header.frame_id,tf2::TimePointZero);
      const double age=(rclcpp::Time(pose.header.stamp)-rclcpp::Time(transform.header.stamp)).seconds();
      if(std::abs(age)>.3) throw;
      geometry_msgs::msg::PoseStamped result;
      tf2::doTransform(pose,result,transform);
      return result.pose;
    }
  }
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                 const std::shared_ptr<tf2_ros::Buffer> tf,
                 const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    MPPIController::configure(parent,name,tf,costmap);
    auto get=parameters_handler_->getParamGetter(name);
    get(requested_task_,"route_task",std::string{});
    get(requested_goal_,"route_goal_xy",std::vector<double>{0.,0.});
  }
  void setPlan(const nav_msgs::msg::Path &path) override {
    MPPIController::setPlan(path);
    std::vector<RoutePoint> points;
    points.reserve(path.poses.size());
    for(const auto &p:path.poses) points.push_back({p.pose.position.x,p.pose.position.y});
    std::lock_guard<std::mutex> lock(*parameters_handler_->getLock());
    const auto &task=requested_task_;
    const bool full=!task.empty() && !points.empty() && requested_goal_.size()==2 &&
      distance(points.back(),{requested_goal_[0],requested_goal_[1]})<=.11;
    heading_.setPath(std::move(points),task==task_ && frame_==path.header.frame_id,full);
    frame_=path.header.frame_id;
    task_=task;
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,const geometry_msgs::msg::Twist &speed,
    nav2_core::GoalChecker *checker) override {
    HeadingScope scope;
    yaw_offset.reset();
    try {
      const auto p=routePose(pose);
      const auto direction=heading_.direction({p.position.x,p.position.y});
      if(direction) yaw_offset=*direction-tf2::getYaw(p.orientation);
    } catch(const tf2::TransformException &) {
      // The base controller still enforces its normal transform and collision checks.
    }
    if(active_!=yaw_offset.has_value()) {
      active_=yaw_offset.has_value();
      RCLCPP_INFO(logger_,"Route heading preference %s",active_ ? "on" : "off");
    }
    return MPPIController::computeVelocityCommands(pose,speed,checker);
  }
};

class RouteHeadingCritic : public mppi::critics::CriticFunction {
  float weight_=3.f;
public:
  void initialize() override {
    parameters_handler_->getParamGetter(name_)(weight_,"cost_weight",3.f);
  }
  void score(mppi::CriticData &data) override {
    if(!enabled_ || !yaw_offset) return;
    const double target=tf2::getYaw(data.state.pose.pose.orientation)+*yaw_offset;
    const auto &yaw=data.trajectories.yaws;
    const auto steps=yaw.shape()[1];
    if(!steps) return;
    for(size_t i=0;i<yaw.shape()[0];++i) {
      const double error=std::abs(std::remainder(target-yaw(i,steps-1),2.*M_PI));
      data.costs(i)+=weight_*std::max(0.,error-.15);
    }
  }
};
}

PLUGINLIB_EXPORT_CLASS(towergo::RouteMPPI, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(towergo::RouteHeadingCritic, mppi::critics::CriticFunction)
