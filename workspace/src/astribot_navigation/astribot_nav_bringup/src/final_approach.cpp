#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_conversions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace towergo
{
// Face the first traversable part of the partial route, not the obstructed goal.
class PrepareApproachTurn : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<nav_msgs::msg::Path>("path"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::OutputPort<nav_msgs::msg::Path>("turn")};
  }

private:
  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped goal, robot;
    const auto tf = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (!getInput("path", path) || path.poses.empty() || !getInput("goal", goal) ||
      !nav2_util::getCurrentPose(robot, *tf, path.header.frame_id, "tg_nav_base", 0.0))
    {
      return BT::NodeStatus::FAILURE;
    }

    using nav2_util::geometry_utils::euclidean_distance;
    // Sub-tolerance changes at the last reachable cell are not a new approach.
    if (euclidean_distance(robot.pose, goal.pose) -
      euclidean_distance(path.poses.back().pose, goal.pose) <= 0.10)
    {
      return BT::NodeStatus::FAILURE;
    }

    // Smac planned from this robot pose. Skip the noisy first grid cell and
    // use the forward tangent (or the endpoint for a short segment).
    auto aim = std::find_if(path.poses.begin(), path.poses.end(), [&](const auto & p) {
      return euclidean_distance(robot.pose, p.pose) >= 0.30;
    });
    const auto & target = aim == path.poses.end() ? path.poses.back() : *aim;
    const double dx = target.pose.position.x - robot.pose.position.x;
    const double dy = target.pose.position.y - robot.pose.position.y;
    robot.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(std::atan2(dy, dx));
    nav_msgs::msg::Path turn;
    turn.header.frame_id = path.header.frame_id;
    robot.header = turn.header;
    turn.poses.push_back(robot);
    setOutput("turn", turn);
    return BT::NodeStatus::SUCCESS;
  }
};

class FinalApproach : public BT::ControlNode
{
public:
  using BT::ControlNode::ControlNode;

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::InputPort<double>("distance"),
            BT::BidirectionalPort<nav_msgs::msg::Path>("route"),
            BT::InputPort<std::string>("route_kind", "full", "Full or temporary approach route"),
            BT::OutputPort<nav_msgs::msg::Path>("final_path")};
  }

private:
  bool onFinalSegment() const
  {
    nav_msgs::msg::Path route;
    std::string kind;
    if (!getInput("route_kind", kind) || kind != "full" ||
      !getInput("route", route) || route.poses.empty())
    {
      return false;
    }
    const auto tf = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    geometry_msgs::msg::PoseStamped robot;
    if (!nav2_util::getCurrentPose(robot, *tf, route.header.frame_id, "tg_nav_base", 0.0)) {
      return false;
    }
    const double threshold = getInput<double>("distance").value();
    const auto distance = [&robot](const auto & pose) {
      return nav2_util::geometry_utils::euclidean_distance(robot.pose, pose.pose);
    };
    if (distance(goal_) > threshold || distance(route.poses.back()) > threshold) {
      return false;
    }
    // A goal across a wall can be nearby while the route around it is still long.
    const auto closest = std::min_element(route.poses.begin(), route.poses.end(),
      [&](const auto & a, const auto & b) {return distance(a) < distance(b);});
    return distance(*closest) + nav2_util::geometry_utils::calculate_path_length(
      route, std::distance(route.poses.begin(), closest)) <= threshold;
  }

  BT::NodeStatus tick() override
  {
    const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal").value();
    if (status() == BT::NodeStatus::IDLE || goal != goal_) {
      haltChildren();
      setOutput("route", nav_msgs::msg::Path{});
      goal_ = goal;
      finishing_ = false;
    }
    setStatus(BT::NodeStatus::RUNNING);

    if (!finishing_) {
      if (!onFinalSegment()) {
        const auto tracking = children_nodes_.at(0)->executeTick();
        if (tracking != BT::NodeStatus::SUCCESS) {
          return tracking;
        }
      }

      haltChild(0);
      // A one-pose path drives Nudge directly to the requested pose. The
      // controller's goal checker must use the latest map-to-odom transform.
      nav_msgs::msg::Path path;
      path.header.frame_id = goal.header.frame_id;
      path.poses = {goal};
      path.poses.front().header = path.header;
      setOutput("final_path", path);
      finishing_ = true;
    }
    return children_nodes_.at(1)->executeTick();
  }

  bool finishing_ = false;
  geometry_msgs::msg::PoseStamped goal_;
};
}  // namespace towergo

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<towergo::PrepareApproachTurn>("PrepareApproachTurn");
  factory.registerNodeType<towergo::FinalApproach>("FinalApproach");
}
