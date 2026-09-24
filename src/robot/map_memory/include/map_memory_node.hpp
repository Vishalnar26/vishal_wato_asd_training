#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"


#include "nav_msgs/msg/occupancy_grid.hpp"    // Occupancy grid message type for costmap data
#include "nav_msgs/msg/odometry.hpp"         // Odometry message type for robot position and movement

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    
    // Subscribes to local costmap updates
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    // Subscribes to robot odometry for tracking movement
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // Timer that periodically checks whether the map should be updated
    rclcpp::TimerBase::SharedPtr timer_;
    // Publishes the accumulated global map
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Stores the most recently received costmap
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    // Stores the most recently received odometry data
    nav_msgs::msg::Odometry latest_odom_;

    // Stores the robot's current position from odometry
    double current_x_;
    double current_y_;

    // Stores the robot's position when the map was last updated
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;

    // Indicates whether the robot has moved far enough to update the map
    bool should_update_map_ = false;
    // Indicates whether a new costmap has been received
    bool costmap_updated_ = false;

    // Handles new costmap data
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    // Handles new odometry data and updates the robot's current position
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // Checks whether the latest costmap should be added to the global map
    void updateMap();

};

#endif 
