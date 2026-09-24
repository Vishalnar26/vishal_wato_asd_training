#include "planner_core.hpp"

// stuff used for A*
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <algorithm>

// used for each point in the final path
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger) {}

// finds a path from the robot to the goal
nav_msgs::msg::Path PlannerCore::findPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Point& start,
    const geometry_msgs::msg::Point& goal)
{
    nav_msgs::msg::Path path;

    // use the same coordinate frame as the map
    path.header.frame_id = map.header.frame_id;

    // converts robot position to a grid cell
    int start_x = static_cast<int>(
        (start.x - map.info.origin.position.x) / map.info.resolution
    );

    int start_y = static_cast<int>(
        (start.y - map.info.origin.position.y) / map.info.resolution
    );

    // converts goal position to a grid cell
    int goal_x = static_cast<int>(
        (goal.x - map.info.origin.position.x) / map.info.resolution
    );

    int goal_y = static_cast<int>(
        (goal.y - map.info.origin.position.y) / map.info.resolution
    );

    // start and goal cells for A*
    CellIndex start_cell(start_x, start_y);
    CellIndex goal_cell(goal_x, goal_y);

    // checks if start and goal are inside the map
    if (start_x < 0 ||
        start_x >= static_cast<int>(map.info.width) ||
        start_y < 0 ||
        start_y >= static_cast<int>(map.info.height) ||
        goal_x < 0 ||
        goal_x >= static_cast<int>(map.info.width) ||
        goal_y < 0 ||
        goal_y >= static_cast<int>(map.info.height)) {

        RCLCPP_WARN(logger_, "Start or goal is outside the map");
        return path;
    }

    // cells A* still needs to check
    std::priority_queue<
        AStarNode,
        std::vector<AStarNode>,
        CompareF
    > open_list;

    // cells A* already checked
    std::unordered_set<CellIndex, CellIndexHash> closed_list;

    // cheapest cost found to each cell
    std::unordered_map<CellIndex, double, CellIndexHash> g_score;

    // remembers where each cell came from
    std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

    // start cell has no travel cost yet
    g_score[start_cell] = 0.0;

    // estimates distance from start to goal
    double start_h =
        std::abs(goal_x - start_x) +
        std::abs(goal_y - start_y);

    // adds the starting cell to the open list
    open_list.push(AStarNode(start_cell, start_h));

    // keeps searching while there are cells left to check
    while (!open_list.empty()) {

        // gets the cell with the lowest f score
        AStarNode current = open_list.top();
        open_list.pop();

        // skips it if we already checked this cell
        if (closed_list.count(current.index) > 0) {
            continue;
        }

        // marks this cell as checked
        closed_list.insert(current.index);

        // stops searching once A* reaches the goal
        if (current.index == goal_cell) {
            break;
        }

        // directions for the 4 neighboring cells
        int directions[4][2] = {
            {1, 0},
            {-1, 0},
            {0, 1},
            {0, -1}
        };

        // checks each neighboring cell
        for (const auto& direction : directions) {

            CellIndex neighbor(
                current.index.x + direction[0],
                current.index.y + direction[1]
            );

            // skips neighbors outside the map
            if (neighbor.x < 0 ||
                neighbor.x >= static_cast<int>(map.info.width) ||
                neighbor.y < 0 ||
                neighbor.y >= static_cast<int>(map.info.height)) {
                continue;
            }

            // gets the neighbor's position in the map data
            int neighbor_index =
                neighbor.y * map.info.width + neighbor.x;

            // skips cells that are obstacles
            if (map.data[neighbor_index] >= 100) {
                continue;
            }

            // skips cells already checked
            if (closed_list.count(neighbor) > 0) {
                continue;
            }

            // cost to reach this neighbor
            double tentative_g =
                g_score[current.index] + 1.0;

            // checks if this is a better way to reach the neighbor
            if (g_score.find(neighbor) == g_score.end() ||
                tentative_g < g_score[neighbor]) {

                // saves the new cheaper cost
                g_score[neighbor] = tentative_g;

                // remembers how we got to this cell
                came_from[neighbor] = current.index;

                // estimates distance from neighbor to goal
                double h =
                    std::abs(goal_x - neighbor.x) +
                    std::abs(goal_y - neighbor.y);

                // total A* score
                double f = tentative_g + h;

                // adds neighbor as a cell to check
                open_list.push(AStarNode(neighbor, f));
            }
        }
    }

    // returns empty path if A* could not reach the goal
    if (closed_list.count(goal_cell) == 0) {
        RCLCPP_WARN(logger_, "Could not find a path to the goal");
        return path;
    }

    // works backwards from goal to start
    std::vector<CellIndex> path_cells;

    CellIndex current_cell = goal_cell;

    while (current_cell != start_cell) {
        path_cells.push_back(current_cell);
        current_cell = came_from[current_cell];
    }

    // adds the starting cell too
    path_cells.push_back(start_cell);

    // changes it from goal-to-start into start-to-goal
    std::reverse(path_cells.begin(), path_cells.end());

    // converts grid cells back into real world positions
    for (const CellIndex& cell : path_cells) {

        geometry_msgs::msg::PoseStamped pose;

        // uses the same coordinate frame as the map
        pose.header.frame_id = map.header.frame_id;

        // converts grid position back to meters
        pose.pose.position.x =
            map.info.origin.position.x +
            (cell.x + 0.5) * map.info.resolution;

        pose.pose.position.y =
            map.info.origin.position.y +
            (cell.y + 0.5) * map.info.resolution;

        // gives the pose a valid rotation
        pose.pose.orientation.w = 1.0;

        // adds this point to the final path
        path.poses.push_back(pose);
    }

    return path;
}

}