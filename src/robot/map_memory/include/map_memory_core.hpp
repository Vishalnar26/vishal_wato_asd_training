#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"  // Occupancy grid type for the global map
#include "nav_msgs/msg/odometry.hpp"  // Odometry type for robot position and orientation

namespace robot{

  class MapMemoryCore {
    public:
      explicit MapMemoryCore(const rclcpp::Logger& logger);
      
      // Integrates a local costmap into the global map using the robot's pose
      void integrateCostmap(
          const nav_msgs::msg::OccupancyGrid& costmap,
          const nav_msgs::msg::Odometry& odom
      );
      // Returns the current global map so it can be published by the ROS node
      const nav_msgs::msg::OccupancyGrid& getGlobalMap() const;
    private:
      rclcpp::Logger logger_;
      // Stores the accumulated global map
      nav_msgs::msg::OccupancyGrid global_map_;
  };

}  

#endif  
