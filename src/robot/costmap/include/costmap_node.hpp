#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void publishMessage();
    

  private:
    robot::CostmapCore costmap_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_; // Subscription to LIDAR data
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_; // Publisher for the occupancy grid costmap

    std::vector<int8_t> grid_;
    int width_ = 300;    
    int height_ = 300; 
    double resolution_ = 0.1;
    double origin_x_ = -15.0; 
    double origin_y_ = -15.0;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;
};

#endif 