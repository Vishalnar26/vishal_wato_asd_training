#include "control_node.hpp"

#include <cmath>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this-> create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));


}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg; 
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_) {
    return;
  }

  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    return;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (current_path_->poses.empty()) {
    return std::nullopt;
  }

  const auto &robot_position = robot_odom_->pose.pose.position;
  for (const auto &pose : current_path_->poses) {
    if (computeDistance(robot_position, pose.pose.position) >= lookahead_distance_) {
      return pose;
    }
  }

  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);
  const auto &robot_position = robot_odom_->pose.pose.position;

  double dx = target.pose.position.x - robot_position.x;
  double dy = target.pose.position.y - robot_position.y;
  double angle_to_target = std::atan2(dy, dx);

  double steering_angle = angle_to_target - robot_yaw;
  steering_angle = std::atan2(std::sin(steering_angle), std::cos(steering_angle));

  double curvature = 2 * std::sin(steering_angle) / lookahead_distance_;

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear_speed_;
  cmd_vel.angular.z = curvature * linear_speed_;

  if (computeDistance(robot_position, current_path_->poses.back().pose.position) < goal_tolerance_) {
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
