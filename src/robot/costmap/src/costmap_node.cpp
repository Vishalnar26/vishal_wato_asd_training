#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
    /*
    string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
    */

    grid_.resize(width_ * height_, 0);

    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  std::fill(grid_.begin(), grid_.end(), 0);

  for (size_t i = 0; i < scan->ranges.size(); i++) {
    double range = scan->ranges[i];

    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    double angle = scan->angle_min + i * scan->angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int grid_x = static_cast<int>((x - origin_x_) / resolution_);
    int grid_y = static_cast<int>((y - origin_y_) / resolution_);

    if (grid_x < 0 || grid_x >= width_ || grid_y < 0 || grid_y >= height_) {
      continue;
    }

    grid_[grid_y * width_ + grid_x] = 100; //100 = "OBSTACLE HERE, DANGER"

    
  }

  for (int gy = 0; gy < height_; gy++) {
    for (int gx = 0; gx < width_; gx++) {
      if (grid_[gy * width_ + gx] != 100) {
        continue; 
      }

      int cell_radius = static_cast<int>(inflation_radius_ / resolution_);
      for (int dy = -cell_radius; dy <= cell_radius; dy++) {
        for (int dx = -cell_radius; dx <= cell_radius; dx++) {
          int nx = gx + dx;
          int ny = gy + dy;
          if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
            continue;
          }

          double d = std::sqrt(dx * dx + dy * dy) * resolution_;
          if (d > inflation_radius_) {
            continue;
          }

          int8_t cost = static_cast<int8_t>(max_cost_ * (1 - d / inflation_radius_));
          int idx = ny * width_ + nx;
          if (cost > grid_[idx]) {
            grid_[idx] = cost;
          }
        }
      }

    }
  }

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.stamp = this->now();
  grid_msg.header.frame_id = "map";
  grid_msg.info.resolution = resolution_;
  grid_msg.info.width = width_;
  grid_msg.info.height = height_;
  grid_msg.info.origin.position.x = origin_x_;
  grid_msg.info.origin.position.y = origin_y_;

  grid_msg.data = grid_; 

  costmap_pub_->publish(grid_msg);

  
}

void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}