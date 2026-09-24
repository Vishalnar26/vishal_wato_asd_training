#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

// used for storing and comparing A* cells
#include <functional>
#include <cstddef>

// message types used by the planner
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace robot
{

// represents one cell in the map grid
struct CellIndex {
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}

  CellIndex() : x(0), y(0) {}

  // checks if two cells are the same
  bool operator==(const CellIndex& other) const {
    return x == other.x && y == other.y;
  }

  // checks if two cells are different
  bool operator!=(const CellIndex& other) const {
    return x != other.x || y != other.y;
  }
};

// lets grid cells be stored in unordered maps
struct CellIndexHash {
  std::size_t operator()(const CellIndex& cell) const {
    return std::hash<int>()(cell.x) ^
           (std::hash<int>()(cell.y) << 1);
  }
};

// stores a cell and its A* score
struct AStarNode {
  CellIndex index;
  double f_score;

  AStarNode(CellIndex idx, double f)
      : index(idx), f_score(f) {}
};

// compares A* nodes by their f score
struct CompareF {
  bool operator()(const AStarNode& a, const AStarNode& b) const {
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // finds a path from the robot to the goal
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