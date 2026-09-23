#include "map_memory_core.hpp"
#include <cmath>

namespace robot
{

  MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
    : logger_(logger) {}

  // Integrates the latest local costmap into the global map using the robot's position and direction  
  void MapMemoryCore::integrateCostmap(
    const nav_msgs::msg::OccupancyGrid& costmap,
    const nav_msgs::msg::Odometry& odom)
  {
    // Initialize the global map to cover the entire simulation environment
    if (global_map_.data.empty()) {

        // Use the same resolution as the local costmap
        global_map_.info.resolution = costmap.info.resolution;

        // Create a 30 meter by 30 meter global map
        global_map_.info.width = 300;
        global_map_.info.height = 300;

        // Set the bottom-left corner of the global map
        global_map_.info.origin.position.x = -15.0;
        global_map_.info.origin.position.y = -15.0;

        // Set a valid orientation for the global map
        global_map_.info.origin.orientation.w = 1.0;

        // Set the global map coordinate frame
        global_map_.header.frame_id = "sim_world";

        // Start every cell as unknown
        global_map_.data.assign(
            global_map_.info.width * global_map_.info.height,
            -1
        );
    }
    // Get the robot's position in the global frame
    double robot_x = odom.pose.pose.position.x;
    double robot_y = odom.pose.pose.position.y;

    // Get the robot's orientation
    double qx = odom.pose.pose.orientation.x;
    double qy = odom.pose.pose.orientation.y;
    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;

    // Convert the robot's orientation into a simple rotation angle
    double robot_direction = std::atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz)
    );

    // Get the size of each costmap cell in meters
    double resolution = costmap.info.resolution;

    // Go through every cell in the local costmap
    for (int row = 0; row < costmap.info.height; row++) {
      for (int col = 0; col < costmap.info.width; col++) {

          // Find this cell's position in the one-dimensional data array
          int index = row * costmap.info.width + col;

          // Get the cost stored in this cell
          int8_t cell_value = costmap.data[index];

          // Ignore unknown cells so existing map data is preserved
          if (cell_value == -1) {
              continue;
          }

          // Convert the cell's row and column into local x and y coordinates
          double local_x = costmap.info.origin.position.x + col * resolution;
          double local_y =costmap.info.origin.position.y + row * resolution;

          // Rotate the local coordinates based on the robot's direction
          double rotated_x =
              local_x * std::cos(robot_direction) - local_y * std::sin(robot_direction);

          double rotated_y =
              local_x * std::sin(robot_direction) + local_y * std::cos(robot_direction);
          
          // Convert the coordinates into a position in the world
          double global_x = robot_x + rotated_x;
          double global_y = robot_y + rotated_y;

          // Convert the global position into a column and row in the global map
          int global_col = static_cast<int>(
              (global_x - global_map_.info.origin.position.x)
              / global_map_.info.resolution);

          int global_row = static_cast<int>(
              (global_y - global_map_.info.origin.position.y)
              / global_map_.info.resolution);

          // Skip the cell if its position is outside the global map
          if (global_col < 0 ||
              global_col >= static_cast<int>(global_map_.info.width) ||
              global_row < 0 ||
              global_row >= static_cast<int>(global_map_.info.height)) {
              continue;
          }

          // Find this cell's position in the global map data array
          int global_index =
              global_row * global_map_.info.width + global_col;

          // Store the new known cost in the global map
          global_map_.data[global_index] = cell_value;

      }
    }
    
  }

  
  // Returns the current global map
  const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const
  {
      return global_map_;
  }
  
} 
