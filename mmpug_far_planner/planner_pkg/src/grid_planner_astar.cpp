#include "grid_planner.h"

using namespace std;

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
        EstimateEuclideanDistance(start_node.x, start_node.y, goal_node.x, goal_node.y) <= 8) {
        // waypoints tolerance
        // ROS_INFO("REACH ONE GOAL.");
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
