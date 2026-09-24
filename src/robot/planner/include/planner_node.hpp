#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

// message types
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {

  public:

    PlannerNode();

  private:

    robot::PlannerCore planner_;

    // planner states
    enum class State {
      WAITING_FOR_GOAL,
      WAITING_FOR_ROBOT_TO_REACH_GOAL
    };

    // starts off waiting for a goal
    State state_ = State::WAITING_FOR_GOAL;

    // global map
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    // goal point
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;

    // robot position
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // planned path
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // checks goal/progress
    rclcpp::TimerBase::SharedPtr timer_;

    // latest map
    nav_msgs::msg::OccupancyGrid current_map_;

    // current goal
    geometry_msgs::msg::PointStamped goal_;

    // latest robot position
    nav_msgs::msg::Odometry latest_odom_;

    // whether we have a goal yet
    bool goal_received_ = false;
    
    // whether we have robot position yet
    bool odom_received_ = false;  

    // callbacks
    void mapCallback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void goalCallback(
        const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void odomCallback(
        const nav_msgs::msg::Odometry::SharedPtr msg);

    // checks if goal was reached
    void timerCallback();

    // checks distance to goal
    bool goalReached();

    // calculates the path
    void planPath();
};

#endif