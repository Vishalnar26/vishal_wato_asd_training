#include "control_node.hpp"

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
  IF (!current_path_ || !robot_odom_) {
    RETURN;
  }

  auto lookahead_point = findLookaheadPoint();
  IF (!lookahead_point) {
    RETURN;
  }

  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
  }

  std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    // WALK THROUGH current_path_->poses ONE BY ONE.
    // FOR EACH POINT, computeDistance() FROM ROBOT CURRENT POSITION
    //   (robot_odom_->pose.pose.position) TO THAT PATH POINT.
    // FIRST POINT WHOSE DISTANCE >= lookahead_distance_ = YOUR PICK.
    // IF NO POINT FAR ENOUGH (WE NEAR END OF PATH), RETURN LAST POINT
    //   OR std::nullopt IF PATH TRULY EMPTY.
  }

  geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    // 1. GET ROBOT YAW: extractYaw(robot_odom_->pose.pose.orientation)
    // 2. FIND ANGLE FROM ROBOT TO target (atan2(dy, dx))
    // 3. STEERING ANGLE = ANGLE FROM ROBOT TO TARGET MINUS ROBOT YAW
    //    (NORMALIZE TO -PI..PI SO ROBOT NOT SPIN WRONG WAY)
    // 4. CURVATURE = 2 * sin(STEERING ANGLE) / lookahead_distance_
    // 5. geometry_msgs::msg::Twist cmd_vel;
    //    cmd_vel.linear.x = linear_speed_;
    //    cmd_vel.angular.z = CURVATURE * linear_speed_;
    // 6. IF DISTANCE TO current_path_->poses.back() < goal_tolerance_,
    //    SET BOTH linear.x AND angular.z TO 0 (WE HOME, STOP CAVE-BOT).
    // RETURN cmd_vel;
  }

  double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    RETURN std::hypot(b.x - a.x, b.y - a.y);   <- PYTHAGORAS. CAVEMAN LOVE TRIANGLE.
  }

  double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    // QUATERNION WEIRD 4-NUMBER ROTATION THING. USE tf2 HELPER:
    //   tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    //   tf2::Matrix3x3 m(q);
    //   double roll, pitch, yaw;
    //   m.getRPY(roll, pitch, yaw);
    //   RETURN yaw;
    // NEED #include "tf2/LinearMath/Matrix3x3.h" AND
    //      #include "tf2/LinearMath/Quaternion.h" FOR THIS.
  }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
