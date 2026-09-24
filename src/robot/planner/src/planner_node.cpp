
#include "planner_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

PlannerNode::PlannerNode()
    : Node("planner"),
      planner_(this->get_logger())
{
    // Receive the accumulated global map
    map_sub_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            10,
            std::bind(
                &PlannerNode::mapCallback,
                this,
                std::placeholders::_1
            )
        );

    // Receive the destination selected in Foxglove
    goal_sub_ =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point",
            10,
            std::bind(
                &PlannerNode::goalCallback,
                this,
                std::placeholders::_1
            )
        );

    // Track the robot's current position
    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered",
            10,
            std::bind(
                &PlannerNode::odomCallback,
                this,
                std::placeholders::_1
            )
        );

    // Send the planned route to the Control node
    path_pub_ =
        this->create_publisher<nav_msgs::msg::Path>(
            "/path",
            10
        );

    // Recheck the route once per second
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&PlannerNode::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Planner node started");
}

void PlannerNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    latest_map_ = *msg;
    map_received_ = true;
}

void PlannerNode::goalCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_ = *msg;
    goal_received_ = true;

    RCLCPP_INFO(
        this->get_logger(),
        "Received goal: (%.2f, %.2f)",
        goal_.point.x,
        goal_.point.y
    );

    planPath();
}

void PlannerNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    latest_odom_ = *msg;
    odom_received_ = true;
}

bool PlannerNode::goalReached() const
{
    if (!goal_received_ || !odom_received_) {
        return false;
    }

    double dx =
        latest_odom_.pose.pose.position.x - goal_.point.x;

    double dy =
        latest_odom_.pose.pose.position.y - goal_.point.y;

    return std::sqrt(dx * dx + dy * dy) < 0.4;
}

void PlannerNode::planPath()
{
    if (!map_received_ || !odom_received_ || !goal_received_) {
        return;
    }

    if (goalReached()) {
        return;
    }

    // All coordinates must use the same reference frame
    if (latest_map_.header.frame_id !=
            latest_odom_.header.frame_id ||
        goal_.header.frame_id !=
            latest_map_.header.frame_id) {
        RCLCPP_WARN(
            this->get_logger(),
            "Map, odometry and goal have different frames"
        );
        return;
    }

    auto path = planner_.findPath(
        latest_map_,
        latest_odom_.pose.pose.position,
        goal_.point
    );

    if (path.poses.empty()) {
        RCLCPP_WARN(
            this->get_logger(),
            "No path available"
        );
        return;
    }

    path.header.stamp = this->now();

    for (auto& pose : path.poses) {
        pose.header.stamp = path.header.stamp;
    }

    path_pub_->publish(path);
}

void PlannerNode::timerCallback()
{
    if (!goal_received_) {
        return;
    }

    if (goalReached()) {
        RCLCPP_INFO(
            this->get_logger(),
            "Robot reached its goal"
        );

        goal_received_ = false;
        return;
    }

    // Update the path using the latest map and robot position
    planPath();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<PlannerNode>()
    );

    rclcpp::shutdown();

    return 0;
}