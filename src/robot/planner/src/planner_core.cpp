
#include "planner_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger) {}

// A* finds a path through free cells while avoiding obstacles
nav_msgs::msg::Path PlannerCore::findPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal)
{
    nav_msgs::msg::Path path;
    path.header = map.header;

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const double resolution = map.info.resolution;

    if (width <= 0 || height <= 0 || resolution <= 0.0 ||
        map.data.size() != static_cast<size_t>(width * height)) {
        RCLCPP_WARN(logger_, "Map is empty or invalid");
        return path;
    }

    const double origin_x = map.info.origin.position.x;
    const double origin_y = map.info.origin.position.y;

    // Convert world coordinates into grid coordinates
    auto toCell = [&](double x, double y) {
        int col = static_cast<int>(
            std::floor((x - origin_x) / resolution));
        int row = static_cast<int>(
            std::floor((y - origin_y) / resolution));

        return std::array<int, 2>{col, row};
    };

    auto start_cell = toCell(start.x, start.y);
    auto goal_cell = toCell(goal.x, goal.y);

    auto insideMap = [&](int col, int row) {
        return col >= 0 && col < width &&
               row >= 0 && row < height;
    };

    if (!insideMap(start_cell[0], start_cell[1]) ||
        !insideMap(goal_cell[0], goal_cell[1])) {
        RCLCPP_WARN(logger_, "Start or goal is outside the map");
        return path;
    }

    auto indexOf = [&](int col, int row) {
        return row * width + col;
    };

    const int start_index =
        indexOf(start_cell[0], start_cell[1]);

    const int goal_index =
        indexOf(goal_cell[0], goal_cell[1]);

    // Unknown cells and cells with high obstacle costs are blocked
    auto isBlocked = [&](int index) {
        int cost = map.data[index];
        return cost < 0 || cost >= 80;
    };

    if (isBlocked(start_index) || isBlocked(goal_index)) {
        RCLCPP_WARN(logger_, "Start or goal is blocked or unknown");
        return path;
    }

    struct SearchNode {
        int index;
        double priority;
    };

    struct CompareNodes {
        bool operator()(
            const SearchNode& a,
            const SearchNode& b) const
        {
            return a.priority > b.priority;
        }
    };

    std::priority_queue<
        SearchNode,
        std::vector<SearchNode>,
        CompareNodes
    > open_set;

    const int total_cells = width * height;

    std::vector<double> distance(
        total_cells,
        std::numeric_limits<double>::infinity()
    );

    std::vector<int> previous(total_cells, -1);
    std::vector<bool> visited(total_cells, false);

    // Manhattan distance estimates how far a cell is from the goal
    auto heuristic = [&](int index) {
        int col = index % width;
        int row = index / width;

        return static_cast<double>(
            std::abs(col - goal_cell[0]) +
            std::abs(row - goal_cell[1])
        );
    };

    distance[start_index] = 0.0;
    open_set.push({start_index, heuristic(start_index)});

    const std::array<std::array<int, 2>, 4> directions = {{
        {{1, 0}},
        {{-1, 0}},
        {{0, 1}},
        {{0, -1}}
    }};

    bool found = false;

    while (!open_set.empty()) {
        int current = open_set.top().index;
        open_set.pop();

        if (visited[current]) {
            continue;
        }

        visited[current] = true;

        if (current == goal_index) {
            found = true;
            break;
        }

        int col = current % width;
        int row = current / width;

        // Check the four neighboring cells
        for (const auto& direction : directions) {
            int next_col = col + direction[0];
            int next_row = row + direction[1];

            if (!insideMap(next_col, next_row)) {
                continue;
            }

            int next = indexOf(next_col, next_row);

            if (visited[next] || isBlocked(next)) {
                continue;
            }

            // Prefer cells with lower obstacle costs
            double movement_cost =
                1.0 + static_cast<double>(map.data[next]) / 100.0;

            double new_distance =
                distance[current] + movement_cost;

            if (new_distance < distance[next]) {
                distance[next] = new_distance;
                previous[next] = current;

                open_set.push({
                    next,
                    new_distance + heuristic(next)
                });
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(logger_, "Could not find a path to the goal");
        return path;
    }

    // Follow the recorded cells backward from goal to start
    std::vector<int> cell_path;

    for (int current = goal_index;
         current != -1;
         current = previous[current]) {
        cell_path.push_back(current);
    }

    std::reverse(cell_path.begin(), cell_path.end());

    // Convert grid cells back into world coordinates
    for (int index : cell_path) {
        int col = index % width;
        int row = index / width;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        pose.pose.position.x =
            origin_x + (col + 0.5) * resolution;

        pose.pose.position.y =
            origin_y + (row + 0.5) * resolution;

        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;

        path.poses.push_back(pose);
    }

    return path;
}

}