/**
 * @file grid_planner.cpp
 * @brief Grid planner class implementation
 * @author Siqi. Edward, Guo
 * @version 1.0
 * @date 2024
 */
#include "grid_planner.h"
using namespace std;

//====================================== Constants =============================================
const float MIN_OFFSET_ANGLE = 0.25;
const float MERGE_DISTANCE = 8.0;

/**
 * @brief Constructor creates a new GridPlanner.
 *
 * @param map_size The size of the map grid.
 * @param map_resolution The resolution of the map grid.
 * @param min_obs_cost The minimum obstacle cost, which is 100 for now.
 */
GridPlanner::GridPlanner(int map_size, float map_resolution, int min_obs_cost)
    : x_size(map_size), y_size(map_size), map_resolution(map_resolution), obstacle_cost(min_obs_cost) {
    this->x_offset = -map_size / 2;
    this->y_offset = -map_size / 2;
    InitializeMap();
}
/**
 * @brief Destructor destroys a GridPlanner.
 *
 * This destructor is responsible for cleaning up the map used by the GridPlanner object.
 * It calls the ResetMap() function to reset the stored map.
 */
GridPlanner::~GridPlanner() { ResetMap(); }

//============================ Global Cost Map Helper Function ================================
//============================
/**
 * @brief Resets the map used by the grid planner.
 *  This function clears the map, removing all previously stored data.
 * @return void
 */
void GridPlanner::ResetMap() {
    this->map.clear();
    return;
}

/**
 * @brief Initializes the map for the grid planner.
 *
 * This function resets the map and initializes it with the obstacle costs.
 * The map is represented as a 2D grid with dimensions x_size and y_size.
 * Each cell in the grid is assigned an obstacle cost equal to half of the minimum obstacle cost
 * specified in the GridPlanner object. (unexplored areas)
 *
 * @return void
 */
void GridPlanner::InitializeMap() {
    ResetMap();
    std::vector<int8_t> data(this->x_size * this->y_size);
    std::fill(data.begin(), data.end(), this->obstacle_cost / 2);
    this->map = data;
    return;
}

/**
 * Check if the given coordinates (x, y) are within the map boundaries.
 *
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return True if the coordinates are within the map boundaries, false otherwise.
 */
auto GridPlanner::IsInMap(int x, int y) -> bool { return (x >= 0 && x < x_size && y >= 0 && y < y_size); }

/**
 * @brief Check if the given pose is in the map.
 *  If the pose is not in the map, the planner will be cancelled. There is a warning message.
 * @param pose The pose to check.
 * @param node The node to store the mapping poses coordinates.
 * @return True if the pose is in the map, false otherwise.
 */
auto GridPlanner::CheckPoseInMap(const geometry_msgs::Pose& pose, Node& node) -> bool {
    node.x = pose.position.x / map_resolution - x_offset;
    node.y = pose.position.y / map_resolution - y_offset;
    if (IsInMap(node.x, node.y) == false) {
        ROS_WARN("CANCELLING PLANNER BECAUSE POSE IS NOT IN THE GRID");
        return false;
    }
    return true;
}

/**
 * @brief Calculates the map index based on the given x and y coordinates. (This is also the Key in Closed list.)
 *
 * @param x The x-coordinate.
 * @param y The y-coordinate.
 * @return The map index.
 */
inline int GridPlanner::GetMapIndex(int x, int y) { return (y * y_size + x); }

/**
 * @brief Calculates the local grid map index based on the given x, y coordinates, and the map 2-D height size.
 */
inline int GridPlanner::GetGridMapIndex(int x, int y, int height) { return (y * height + x); }

inline int GridPlanner::GetMapIndex25D(int x, int y, float theta) { return (t * y_size * x_size + y * x_size + x); }

/**
 * @brief Retrieves the map data.
 *
 * This function is used to retrieve the map data stored in the GridPlanner.
 *  The map data will be put into the vector map_data.
 *
 * @param map_data The vector to store the retrieved map.
 */
void GridPlanner::GetMap(std::vector<int8_t>& map_data) {
    map_data = this->map;
    return;
}

/**
 * @brief Updates the global cost map with the given occupancy grid and origin point.
 *
 * This function iterates over the grid and checks if each cell is within the map boundaries.
 * Each cell will be updated if it has not been updated before in terms of the local's map.
 * Then, the cell will only update the obstacle cost when there is an obstacle in it.
 *
 * For now the filtering function does not work well.
 *
 * @param grid The occupancy grid to update the map with.
 * @param origin_point The origin point in local planner's map, used to calculate the map coordinates.
 */
void GridPlanner::UpdateMap(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& origin_point) {
    int x1 = std::floor(origin_point.x / map_resolution) - x_offset;
    int y1 = std::floor(origin_point.y / map_resolution) - y_offset;
    for (int i = 2; i < grid.info.width - 3; i++) {
        for (int j = 2; j < grid.info.height - 3; j++) {
            if (IsInMap(x1 + i, y1 + j)) {
                int current_idx = GetMapIndex(x1 + i, y1 + j);
                int grid_idx = GetGridMapIndex(i, j, (int)grid.info.height);
                if (this->map[current_idx] == obstacle_cost / 2) {
                    this->map[current_idx] = (int8_t)grid.data.at(grid_idx);
                } else {
                    if (grid.data.at(grid_idx) != obstacle_cost / 2 && grid.data.at(grid_idx) != 0) {
                        this->map[current_idx] = (int8_t)grid.data.at(grid_idx);
                    }
                }
            }
        }
    }
}

//============================ Waypoints Update Function ================================
//============================
/**
 * @brief Update the waypoints for the planner.
 *  The waypoints are the target poses to reach.
 * @param waypoints The array of waypoints to update.
 */
void GridPlanner::UpdateWaypoints(const geometry_msgs::PoseArray& waypoints) {
    if (waypoints.poses.size() == 0) {
        ROS_WARN("NO WAYPOINTS TO UPDATE");
        return;
    }
    if (ros::Time::now().toSec() > dynamic_waypoints.header.stamp.toSec()) {
        dynamic_waypoints = waypoints;
        dynamic_waypoints.header.stamp = ros::Time::now();
        ROS_INFO("NEW INPUT WAYPOINTS UPDATED.");
    }
}

/**
 * @brief Get the dynamic waypoints for the planner.
 *  The dynamic waypoints are the target poses to reach.
 * @param waypoints The array of waypoints to update.
 */
void GridPlanner::GetDynamicWaypoints(geometry_msgs::PoseArray& waypoints) { waypoints = dynamic_waypoints; }

//============================ A* Searching Algorithm Helper Function ================================
//============================
/**
 * @brief Function to estimate the euclidean distance between two points
 *     formula: sqrt(dx^2 + dy^2)
 *  The Octile Distance dx+dy-min(dx, dy) deprecated in this version.
 *
 * @param curr_x The x-coordinate of the current point.
 * @param curr_y The y-coordinate of the current point.
 * @param goal_x The x-coordinate of the goal point.
 * @param goal_y The y-coordinate of the goal point.
 * @return The estimated euclidean distance between these two points.
 */
auto GridPlanner::EstimateEuclideanDistance(int curr_x, int curr_y, int goal_x, int goal_y) -> float {
    int delta_x = abs(curr_x - goal_x);
    int delta_y = abs(curr_y - goal_y);
    return sqrt(delta_x * delta_x + delta_y * delta_y);
}

/**
 * @brief Set the cost of the A* node.
 *  The total cost is calculated by the formula f = g + h.
 * @param nodeptr The node to set the cost.
 * @param g The cost from the start node to the current node.
 * @param h The heuristic cost from the current node to the goal node.
 */
void GridPlanner::SetAstarCost(Nodeptr& nodeptr, float g, float h) {
    nodeptr->g = g;
    nodeptr->h = h;
    nodeptr->f = g + h;
}

/**
 * @brief Calculate the angle between two poses.
 *  The angle is calculated by the formula atan2(y2 - y1, x2 - x1).
 * @param pose_1 The first pose node.
 * @param pose_2 The second pose node.
 * @return The angle between the two poses. The direction in this step.
 */
auto GridPlanner::CalculateAngle(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2) -> float {
    return atan2(pose_2.position.y - pose_1.position.y, pose_2.position.x - pose_1.position.x);
}

//============================ Global Planning with A* Algorithm ================================
//============================
/**
 * @brief Function to plan in a naive way
 *  Open list is a priority queue, with FIFO order. (usually implemented as a min-heap or priority queue)
 *      From the start node, try each 8 directions to find the satisfied successor node in this path.
 *      Then push it into the queue. BFS-like searching. Ensure the path is not overlapped.
 *  After finding one path (or more), backtracking to find the final path with the minimum cost.
 *      To decide whether two waypoints are merged, the angle between them should be larger than a threshold 0.25.
 *      The distance between them should be larger than 8.0.
 */
auto GridPlanner::PlanWithAstar(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target,
                                geometry_msgs::PoseArray& plan) -> int {
    if (CheckPoseInMap(robot_pose, start_node) == false || CheckPoseInMap(target, goal_node) == false) {
        return -1;
    }
    if (abs(EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) - timer_distance) >= 1) {
        plan_timer = ros::Time::now();
    }

    timer_distance = EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y);
    ROS_INFO_STREAM("To Next Input Waypoint Distance: " << EstimateEuclideanDistance(start_node.x, start_node.y,
                                                                                     goal_node.x, goal_node.y));
    if (start_node == goal_node ||
        EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) <= 10) {
        ROS_INFO("REACH ONE GOAL.");
        return 0;
    }

    CLOSED_LIST closed_list;
    OPEN_LIST open_list;
    int goal_key, curr_key;
    bool pathFound = false;

    Nodeptr start = std::make_shared<Node>(start_node.x, start_node.y);
    start->key = GetMapIndex(start_node.x, start_node.y);
    start->time = max_steps;
    SetAstarCost(start, 0, 0);
    start->parent = nullptr;

    open_list.push(start);
    while (!open_list.empty() && closed_list.count(GetMapIndex(goal_node.x, goal_node.y)) == 0) {
        Nodeptr curr_node = open_list.top();
        open_list.pop();

        curr_key = GetMapIndex(curr_node->x, curr_node->y);
        if (closed_list.count(curr_key) == 0) {
            closed_list.insert({curr_key, curr_node});
            if (*curr_node == goal_node) {
                goal_key = curr_key;
                pathFound = true;
                ROS_INFO_STREAM("PATH FOUND WITH STEPS " << curr_node->time << " LEFT.");
                break;
            }
            for (int dir = 0; dir < 8; dir++) {
                int newx = curr_node->x + dX[dir];
                int newy = curr_node->y + dY[dir];
                int newt = curr_node->time - 1;

                bool expand = false;
                int new_key = GetMapIndex(newx, newy);
                if (closed_list.count(new_key) == 0 || closed_list.at(new_key)->time < newt) {
                    expand = true;
                }
                if (expand && IsInMap(newx, newy)) {
                    int new_cost_from_map = (int)map[GetMapIndex(newx, newy)];
                    if (new_cost_from_map >= 0 && new_cost_from_map < obstacle_cost) {
                        float g_s_dash = curr_node->g + new_cost_from_map + sqrt(dX[dir] * dX[dir] + dY[dir] * dY[dir]);
                        float h_s_dash = EstimateEuclideanDistance(newx, newy, goal_node.x, goal_node.y);
                        Nodeptr successor = std::make_shared<Node>(newx, newy);
                        successor->parent = curr_node;
                        SetAstarCost(successor, g_s_dash, h_s_dash);
                        open_list.push(successor);
                    }
                }
            }
            // 8 directions done
        }
    }

    int path_length = 0;
    geometry_msgs::PoseArray best_path;
    geometry_msgs::Pose last_waypoint_2, last_waypoint_1;
    best_path.poses.reserve(x_size + y_size);

    if (!pathFound) {
        ROS_WARN("NO PATH FOUND.");
        return -1;
    }
    if (pathFound) {
        ROS_INFO("BACKTRACKING..");

        path_length = 0;
        Nodeptr backtrackNode = closed_list.at(goal_key);
        while (backtrackNode->parent != NULL) {
            geometry_msgs::Pose wp;
            wp.position.x = (backtrackNode->x + x_offset) * map_resolution;
            wp.position.y = (backtrackNode->y + y_offset) * map_resolution;
            wp.position.z = 0;
            if (path_length <= 1) {
                best_path.poses.push_back(wp);
                if (path_length == 0) {
                    last_waypoint_2 = wp;
                } else {
                    last_waypoint_1 = wp;
                }
            } else {
                float prev_dir = CalculateAngle(last_waypoint_2, last_waypoint_1);
                float curr_dir = CalculateAngle(last_waypoint_1, wp);
                float distance = EstimateEuclideanDistance(last_waypoint_1.position.x, last_waypoint_1.position.y,
                                                           wp.position.x, wp.position.y);
                if (abs(Wrap2PI(prev_dir - curr_dir)) > MIN_OFFSET_ANGLE || distance > MERGE_DISTANCE) {
                    best_path.poses.push_back(wp);
                    last_waypoint_2 = last_waypoint_1;
                    last_waypoint_1 = wp;
                }
            }
            path_length++;
            backtrackNode = backtrackNode->parent;
        }

        plan.poses = best_path.poses;
        std::reverse(plan.poses.begin(), plan.poses.end());
    }
    return path_length;
}

/**
 * @brief Function to plan with Theta* Algorithm
 *  A line-of-sight check (Bresenham's algorithm or similar)
 */
bool LineOfSight(const Nodeptr& start, const Nodeptr& end) {
    int x0 = start->x, y0 = start->y;
    int x1 = end->x, y1 = end->y;
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (map[GetMapIndex(x0, y0)] >= obstacle_cost) return false;  // No line of sight
        if (x0 == x1 && y0 == y1) return true;

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

/**
 * @brief Function to plan with 2.5 D Enhanced A* Algorithm
 *
 * @ref: "Theta*: Any-Angle Path Planning on Grids"
 * @ref: http://www.gameaipro.com/GameAIPro2/GameAIPro2_Chapter16_Theta_Star_for_Any-Angle_Pathfinding.pdf
 * @ref: https://arxiv.org/pdf/1401.3843
 * @ref: https://news.movel.ai/theta-star?x-host=news.movel.ai
 * @ref: https://neuro.bstu.by/ai/To-dom/My_research/Papers-2.0/Closed-loop-path-planning/aaai07a.pdf
 *
 * @ref: Lazy Theta* https://www.mdpi.com/2076-3417/12/20/10601
 * @ref: https://www.sciencedirect.com/science/article/pii/S221491472200006X
 *
 */
auto GridPlanner::PlanWithAstar25D(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target,
                                   geometry_msgs::PoseArray& plan) -> int {
    if (CheckPoseInMap(robot_pose, start_node) == false || CheckPoseInMap(target, goal_node) == false) {
        return -1;
    }
    if (abs(EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) - timer_distance) >= 1) {
        plan_timer = ros::Time::now();
    }

    timer_distance = EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y);
    if (start_node == goal_node ||
        EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) <= 10) {
        return 0;
    }

    CLOSED_LIST closed_list;
    OPEN_LIST open_list;
    int goal_key, curr_key;
    bool pathFound = false;

    Nodeptr start = std::make_shared<Node>(start_node.x, start_node.y);
    start->key = GetMapIndex(start_node.x, start_node.y);
    start->time = max_steps;
    SetAstarCost(start, 0, 0);
    start->parent = nullptr;

    open_list.push(start);
    while (!open_list.empty() && closed_list.count(GetMapIndex(goal_node.x, goal_node.y)) == 0) {
        Nodeptr curr_node = open_list.top();
        open_list.pop();

        curr_key = GetMapIndex(curr_node->x, curr_node->y);
        if (closed_list.count(curr_key) == 0) {
            closed_list.insert({curr_key, curr_node});
            if (*curr_node == goal_node) {
                goal_key = curr_key;
                pathFound = true;
                break;
            }
            for (int dir = 0; dir < 8; dir++) {
                int newx = curr_node->x + dX[dir];
                int newy = curr_node->y + dY[dir];
                int newt = curr_node->time - 1;

                bool expand = false;
                int new_key = GetMapIndex(newx, newy);
                if (closed_list.count(new_key) == 0 || closed_list.at(new_key)->time < newt) {
                    expand = true;
                }
                if (expand && IsInMap(newx, newy)) {
                    int new_cost_from_map = (int)map[GetMapIndex(newx, newy)];
                    if (new_cost_from_map >= 0 && new_cost_from_map < obstacle_cost) {
                        float g_s_dash = curr_node->g + new_cost_from_map + sqrt(dX[dir] * dX[dir] + dY[dir] * dY[dir]);
                        float h_s_dash = EstimateEuclideanDistance(newx, newy, goal_node.x, goal_node.y);
                        Nodeptr successor = std::make_shared<Node>(newx, newy);
                        successor->parent = curr_node;
                        SetAstarCost(successor, g_s_dash, h_s_dash);
                        open_list.push(successor);
                    }
                }
            }
        }
    }

    int path_length = 0;
    geometry_msgs::PoseArray best_path;
    geometry_msgs::Pose last_waypoint_2, last_waypoint_1;
    best_path.poses.reserve(x_size + y_size);

    if (!pathFound) {
        return -1;
    }
    if (pathFound) {
        ROS_INFO("BACKTRACKING..");
        path_length = 0;
        Nodeptr backtrackNode = closed_list.at(goal_key);
        while (backtrackNode->parent != NULL) {
            geometry_msgs::Pose wp;
            wp.position.x = (backtrackNode->x + x_offset) * map_resolution;
            wp.position.y = (backtrackNode->y + y_offset) * map_resolution;
            wp.position.z = 0;
            if (path_length <= 1) {
                best_path.poses.push_back(wp);
                if (path_length == 0) {
                    last_waypoint_2 = wp;
                } else {
                    last_waypoint_1 = wp;
                }
            } else {
                float prev_dir = CalculateAngle(last_waypoint_2, last_waypoint_1);
                float curr_dir = CalculateAngle(last_waypoint_1, wp);
                float distance = EstimateEuclideanDistance(last_waypoint_1.position.x, last_waypoint_1.position.y,
                                                           wp.position.x, wp.position.y);
                if (abs(Wrap2PI(prev_dir - curr_dir)) > MIN_OFFSET_ANGLE || distance > MERGE_DISTANCE) {
                    best_path.poses.push_back(wp);
                    last_waypoint_2 = last_waypoint_1;
                    last_waypoint_1 = wp;
                }
            }
            path_length++;
            backtrackNode = backtrackNode->parent;
        }

        plan.poses = best_path.poses;
        std::reverse(plan.poses.begin(), plan.poses.end());
    }
    return path_length;
}

//============================ Debugging and Testing ================================
//============================
/**
 * @brief Log the information of the GridPlanner object.
 */
void GridPlanner::PrintInfo() {
    ROS_INFO_STREAM("Map size: " << map.size());
    ROS_INFO_STREAM("Robot Position: (" << start_node.x << ", " << start_node.y << ")");
    ROS_INFO_STREAM("x_offset: " << x_offset << " y_offset: " << y_offset);
}
