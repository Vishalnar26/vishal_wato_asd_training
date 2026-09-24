
#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();

private:
    robot::PlannerCore planner_;

    // Receive the global map, goal and robot position
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
        map_sub_;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr
        goal_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    // Publish the path for the Control node
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        path_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_map_;
    nav_msgs::msg::Odometry latest_odom_;
    geometry_msgs::msg::PointStamped goal_;

    bool map_received_ = false;
    bool odom_received_ = false;
    bool goal_received_ = false;

    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void goalCallback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

    void planPath();
    void timerCallback();
    bool goalReached() const;
};

#endif