
#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace robot {

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Find a path from the robot's position to the goal using A*
    nav_msgs::msg::Path findPath(
        const nav_msgs::msg::OccupancyGrid& map,
        const geometry_msgs::msg::Point& start,
        const geometry_msgs::msg::Point& goal
    );

private:
    rclcpp::Logger logger_;
};

}

#endif