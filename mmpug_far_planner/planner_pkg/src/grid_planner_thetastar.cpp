#include <cmath>
#include "grid_planner.h"

using namespace std;

/**
 * @brief Function to plan with Theta* Algorithm (Any-Angle Path Planning on Grids)
 *  A line-of-sight check (Bresenham's algorithm)
 *
 * @ref: "Theta*: Any-Angle Path Planning on Grids"
 *
 * @param start: the start end of the line to check
 * @param end: the end end of the line to check
 * @return bool: true if there is a line of sight between the two points, false otherwise
 */
auto GridPlanner::LineOfSight(const Nodeptr& start, const Nodeptr& end) -> bool {
    /** Extract the point from Node */
    int x0 = start->x;
    int y0 = start->y;
    int x1 = end->x;
    int y1 = end->y;
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int max_iter_step = 2 * (dx + dy + 1);

    /** Direction indicator */
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (int i = 0; i < max_iter_step; i++) {
        // if (map[GetMapIndex(x0, y0)] >= obstacle_cost) return false;
        if (configuration_space_map[GetMapIndex(x0, y0)] >= obstacle_cost) return false; /** No line of sight */

        if (x0 == x1 && y0 == y1) {
            return true;
        }

        int e2 = 2 * err;
        if (e2 > -dy) err -= dy;
        if (e2 > -dy) x0 += sx;
        if (e2 < dx) err += dx;
        if (e2 < dx) y0 += sy;
    }
    return false;
}

/**
 * @brief Function to filter expansion directions (Any-Angle Path Planning on Cells)
 *
 * During A* (variants) search, there are 8 directions.
 *   index  0: left up      1: left         2: left down
 *          3: up           4: down
 *          5: right up     6: right        7: right down
 *
 *     yaw could be the nearest 45 degree orientation
 *
 *      ----------------------------
 *      |   0    |    3   |    5   |
 *      |        |        |        |
 *      ----------------------------
 *      |   1    |        |    6   |
 *      |        |        |        |
 *      ----------------------------
 *      |   2    |    4   |    7   |
 *      |        |        |        |
 *      ----------------------------
 *
 * @param direction: the direction index
 * @param yaw: the orientation of the vehicle (can be other angles which need to be checked if it's valid)
 *
 * @return bool: true if the direction is valid, false otherwise
 */
auto GridPlanner::OrientationConstraint(int direction, int yaw) -> bool {
    // ROS_INFO_STREAM("Orientation Constraint with current yaw: " << this->curr_yaw);

    switch (yaw) {
        case 0:
        case 180:
        case -180:
            return direction == 0 || direction == 1 || direction == 2 || direction == 5 || direction == 6 ||
                   direction == 7;
        case 90:
        case -90:
            return direction == 0 || direction == 3 || direction == 5 || direction == 2 || direction == 4 ||
                   direction == 7;
        case 45:
        case -135:
            return direction == 1 || direction == 2 || direction == 3 || direction == 4 || direction == 5 ||
                   direction == 6;
        case -45:
        case 135:
            return direction == 0 || direction == 1 || direction == 3 || direction == 4 || direction == 6 ||
                   direction == 7;
        default:
            ROS_WARN("Invalid orientation constraint");
            break;
    }

    return false;
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
 * (1) convert the A* to Theta*
 * (2) check the orientation constraint
 *
 * @param robot_pose: the current pose of the robot
 * @param target: the target pose
 * @param plan: the planned path
 * @return int: the length of the planned path
 */
auto GridPlanner::PlanWithThetAstar(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target,
                                    geometry_msgs::PoseArray& plan) -> int {
    if (CheckPoseInMap(robot_pose, start_node) == false || CheckPoseInMap(target, goal_node) == false) {
        return -1;
    }
    if (abs(EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) - timer_distance) >= 1) {
        plan_timeout_timer = ros::Time::now();
    }

    timer_distance = EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y);
    if (start_node == goal_node ||
        EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) <= reach_goal_threshold_) {
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

                /** 2.5 D */
                if (expand) {
                    expand = OrientationConstraint(dir, this->curr_yaw);
                }

                if (expand && IsInMap(newx, newy)) {
                    // int new_cost_from_map = (int)map[GetMapIndex(newx, newy)];
                    int new_cost_from_map = (int)configuration_space_map[GetMapIndex(newx, newy)];

                    if (new_cost_from_map >= 0 && new_cost_from_map < obstacle_cost) {
                        float g_s_dash = curr_node->g + new_cost_from_map + sqrt(dX[dir] * dX[dir] + dY[dir] * dY[dir]);
                        float h_s_dash = EstimateEuclideanDistance(newx, newy, goal_node.x, goal_node.y);

                        Nodeptr successor = std::make_shared<Node>(newx, newy);
                        successor->parent = curr_node;

                        /**
                         * Theta* variant check:
                         * if there's line of sight between curr_node->parent and successor
                         */
                        if (curr_node->parent != nullptr) {
                            /** simplified theta* */

                            if (LineOfSight(curr_node->parent, successor)) {
                                /**
                                 * If the map is guaranteed to be initialized as the same orientation as the vehicle
                                 */
                                // Calculate the differences
                                double dy = successor->y - curr_node->parent->y;
                                double dx = successor->x - curr_node->parent->x;

                                double theta = atan2(dy, dx);
                                double theta_degrees = theta * (180.0 / M_PI);

                                theta_degrees = std::round(theta_degrees / 45.0) * 45.0;

                                /** 2.5 D */
                                if (OrientationConstraint(dir, (int)theta_degrees)) {
                                    successor->parent = curr_node->parent;
                                }
                            }
                        }

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