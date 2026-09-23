#include "map_memory_node.hpp"

#include <cmath>
#include <chrono>

// Creates the Map Memory node and sets up its subscribers, publisher, and timer
MapMemoryNode::MapMemoryNode()
    : Node("map_memory"),
      map_memory_(robot::MapMemoryCore(this->get_logger()))
{
    // Subscribe to robot odometry updates
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered",
        10,
        std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1)
    );

    // Subscribe to local costmap updates
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap",
        10,
        std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1)
    );

    // Publish the accumulated global map on the /map topic
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/map",
        10
    );

    // Check every second whether the global map should be updated
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MapMemoryNode::updateMap, this)
    );
}


// Handles new costmap data
void MapMemoryNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // Store the latest costmap
    latest_costmap_ = *msg;

    // Indicate that a new costmap is available
    costmap_updated_ = true;
}


// Handles new odometry data and tracks robot movement
void MapMemoryNode::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Store the latest odometry data
    latest_odom_ = *msg;

    // Get the robot's current position
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // Calculate the distance from the last map update position
    double distance = std::sqrt(
        std::pow(current_x - last_update_x_, 2) +
        std::pow(current_y - last_update_y_, 2)
    );

    // Check if the robot has moved at least 5 meters
    if (distance >= 5.0) {
        last_update_x_ = current_x;
        last_update_y_ = current_y;
        should_update_map_ = true;
    }
}


// Checks whether the latest costmap should be added to the global map
void MapMemoryNode::updateMap()
{
    // Only update when the robot has moved far enough
    // and a new costmap is available
    if (should_update_map_ && costmap_updated_) {

        // Add the latest costmap information to the global map
        map_memory_.integrateCostmap(
            latest_costmap_,
            latest_odom_
        );

        // Publish the updated global map
        map_pub_->publish(map_memory_.getGlobalMap());

        // Reset the flags after the update is complete
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}


// Starts the ROS2 node
int main(int argc, char ** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Run the Map Memory node and process callbacks
    rclcpp::spin(std::make_shared<MapMemoryNode>());

    // Shut down ROS2 when the node stops
    rclcpp::shutdown();

    return 0;
}