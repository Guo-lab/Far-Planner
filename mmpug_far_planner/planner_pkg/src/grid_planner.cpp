/**
 * @file grid_planner.cpp
 * @brief Grid planner class implementation
 * @author Siqi. Edward, Guo
 * @version 2.0
 * @date 2024
 */

#include "grid_planner.h"

using namespace std;

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
    this->curr_yaw = 0.0;

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
    this->configuration_space_map.clear();

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
        ROS_WARN("CANCELLING PLANNING BECAUSE POSE IS NOT IN THE GRID");

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
int GridPlanner::GetMapIndex(int x, int y) { return (y * y_size + x); }

/**
 * @brief Calculates the local grid map index based on the given x, y coordinates, and the map 2-D height size.
 */
int GridPlanner::GetGridMapIndex(int x, int y, int height) { return (y * height + x); }

/**
 * @brief Retrieves the map data.
 *
 * This function is used to retrieve the map data stored in the GridPlanner.
 *  The map data will be put into the vector map_data.
 *
 * @param map_data The vector to store the retrieved map.
 */
void GridPlanner::GetMap(std::vector<int8_t>& map_data) {
    map_data = this->configuration_space_map;

    return;
}

/**
 * @brief Thicken the obstacles in the map. Convert Workspace To Configuration Space.
 *  This function is used to thicken the obstacles in the map.
 *
 * @param map_data_to_thicken The map data to dilate.
 * @return void
 */
void GridPlanner::ThickenObstacles(std::vector<int8_t, std::allocator<int8_t>>& map_data_to_thicken) {
    // CV_8S indicates the data type (8-bit signed integer)
    cv::Mat map_mat = cv::Mat(this->y_size, this->x_size, CV_8U, map_data_to_thicken.data()).clone();

    // Perform dilation or any other desired operation to thicken obstacles
    cv::Mat dilated_map;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * robot_radius_ + 1, 2 * robot_radius_ + 1));

    /**
     * @ref: https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html
     *
     * As the kernel B is scanned over the image, we compute the maximal pixel value overlapped by B and
     *  replace the image pixel in the anchor point position with that maximal value.
     */
    cv::dilate(map_mat, dilated_map, element);

    // Convert back to std::vector<int8_t>
    std::vector<int8_t> vector_map(dilated_map.data, dilated_map.data + dilated_map.total());
    this->configuration_space_map = vector_map;

    return;
}

/**
 * @brief Updates the global cost map with the given occupancy grid and origin point.
 *
 * This function iterates over the grid and checks if each cell is within the map boundaries.
 * Each cell will be updated if it has not been updated before in terms of the local's map.
 * Then, the cell will only update the obstacle cost when there is an obstacle in it.
 *
 * In this way, the robot could avoid the dynamically obstacles in the map and reuse the space when the obstacles leave.
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

                if (this->map[current_idx] == this->obstacle_cost / 2) {
                    this->map[current_idx] = (int8_t)grid.data.at(grid_idx);

                } else {
                    if (grid.data.at(grid_idx) == this->obstacle_cost) {
                        this->map[current_idx] = (int8_t)grid.data.at(grid_idx);
                    }

                    // ROS_INFO_STREAM("grid.data.at(grid_idx): " << grid.data.at(grid_idx));
                    // ROS_INFO_STREAM("ground_occ_grid.data.at(grid_idx): " << ground_occ_grid.data.at(grid_idx));

                    if (grid.data.at(grid_idx) == 0 && ground_occ_grid.data.at(grid_idx) == 0) {
                        // ROS_INFO_STREAM("ground_occ_grid.data.at (x,y) " << x1 + i << ", " << y1 + j);

                        this->map[current_idx] = (int8_t)grid.data.at(grid_idx);
                    }
                }
            }
        }
    }

    ThickenObstacles(this->map);

    return;
}

/**
 * @brief Update the map based on the ground occupancy grid.
 *  The Ground occupancy grid will be used for filtering out removed-obstacles and updating the cost map dynamically.
 *
 * @param grid The ground occupancy grid to update the map with. It consists of the ground points.
 * @return void
 */
void GridPlanner::UpdateMapBasedOnGround(const nav_msgs::OccupancyGrid& grid) {
    ground_occ_grid = grid;

    return;
}

//============================ Waypoints Update Function ================================
//============================
/**
 * @brief Update the waypoints for the planner. The waypoints are the target poses to reach.
 *
 * @param waypoints The array of waypoints to update.
 * @return void
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

        return;
    }
}

/**
 * @brief Get the dynamic waypoints for the planner. The dynamic waypoints are the target poses to reach.
 *
 * @param waypoints The array of waypoints to update.
 * @return void
 */
void GridPlanner::GetDynamicWaypoints(geometry_msgs::PoseArray& waypoints) {
    waypoints = dynamic_waypoints;

    return;
}

//============================ A* Searching Algorithm Helper Function ================================
//============================
/**
 * @brief Function to estimate the euclidean distance between two points with the formula: sqrt(dx^2 + dy^2)
 *
 * The Octile Distance dx+dy-min(dx, dy) deprecated in this version.
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
 * @brief Set the cost of the A* node. The total cost is calculated by the formula f = g + h.
 *
 * @param nodeptr The node to set the cost.
 * @param g The cost from the start node to the current node.
 * @param h The heuristic cost from the current node to the goal node.
 * @return void
 */
void GridPlanner::SetAstarCost(Nodeptr& nodeptr, float g, float h) {
    nodeptr->g = g;
    nodeptr->h = h;
    nodeptr->f = g + h;

    return;
}

/**
 * @brief Calculate the angle between two poses. The angle is calculated by the formula atan2(y2 - y1, x2 - x1).
 *
 * @param pose_1 The first pose node.
 * @param pose_2 The second pose node.
 * @return The angle between the two poses. The direction in this step.
 */
auto GridPlanner::CalculateAngle(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2) -> float {
    return atan2(pose_2.position.y - pose_1.position.y, pose_2.position.x - pose_1.position.x);
}
