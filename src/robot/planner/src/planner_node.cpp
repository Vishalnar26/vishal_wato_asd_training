#include "planner_node.hpp"

#include <chrono>
#include <cmath>

// sets up planner node
PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(robot::PlannerCore(this->get_logger()))
{
    // global map
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map",
        10,
        std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
    );

    // goal point
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point",
        10,
        std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
    );

    // robot position
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered",
        10,
        std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1)
    );

    // publishes planned path
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/path",
        10
    );

    // checks planner every half second
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&PlannerNode::timerCallback, this)
    );
}


// saves newest map
void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = *msg;

    // replan if map changes while going to goal
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
        planPath();
    }
}


// saves new goal
void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_ = *msg;
    goal_received_ = true;

    // switch state since we now have a goal
    state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;

    planPath();
}


// saves newest robot position
void PlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = *msg;
    odom_received_ = true;
}


// checks if robot is close enough to goal
bool PlannerNode::goalReached()
{
    // distance between robot and goal
    double dx = goal_.point.x - latest_odom_.pose.pose.position.x;
    double dy = goal_.point.y - latest_odom_.pose.pose.position.y;

    double distance = std::sqrt(
        dx * dx + dy * dy
    );

    return distance < 0.5;
}


// checks robot progress
void PlannerNode::timerCallback()
{
    if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {

        // go back to waiting once goal is reached
        if (goalReached()) {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");

            state_ = State::WAITING_FOR_GOAL;
            goal_received_ = false;
        }
    }
}


// calculates path
// calculates and publishes the path
void PlannerNode::planPath()
{
    // makes sure we have a map and goal
    if (current_map_.data.empty() || !goal_received_ || !odom_received_) {
        return;
    }

    // gets robot's current position
    geometry_msgs::msg::Point start;
    start.x = latest_odom_.pose.pose.position.x;
    start.y = latest_odom_.pose.pose.position.y;

    // gets goal position
    geometry_msgs::msg::Point goal;
    goal.x = goal_.point.x;
    goal.y = goal_.point.y;

    // runs A*
    nav_msgs::msg::Path path =
        planner_.findPath(current_map_, start, goal);

    // publishes the path
    path_pub_->publish(path);
}


// starts ROS2 node
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}