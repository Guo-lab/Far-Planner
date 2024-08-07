#ifndef GRID_PLANNER_H
#define GRID_PLANNER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#ifndef MIN
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif

//====================================== Constants =============================================
const float MIN_OFFSET_ANGLE = 0.25;
const float MERGE_DISTANCE = 8.0;

const int LEFT = -1;
const int RIGHT = 1;
const int UP = -1;
const int DOWN = 1;
const int STAY = 0;

const int reach_goal_threshold_ = 4;
const int robot_radius_ = 2;

/**
 * =============================================================================================
 *
 *                                      Node Structure
 *
 * =============================================================================================
 *
 * @brief Node structure for representing a state in the search algorithm.
 */
struct Node {
    /**
     * @brief Node fields.  key: The index of the node in the map.
     *                      x, y: The x, y coordinate of the node.
     */
    int key, x, y, time;
    /**
     *  g: The cost to reach the node, the cost of the path from the start node to the current node
     *
     *  h: The heuristic cost to reach the goal from the node. It estimates the cost of the cheapest path from the
     *      current node to the goal node.
     *
     *  f: The total cost of the node. (g + h). Cost to determine which node to explore
     *      next. It prioritizes nodes with lower f values.
     */
    float g, h, f;

    /**
     * @brief The parent node of the current node, which is a shared pointer.
     */
    std::shared_ptr<Node> parent;

    /**
     * @brief Constructor for Node.
     *
     * @param x The x coordinate of the node.
     * @param y The y coordinate of the node.
     */
    Node(int x, int y) : x(x), y(y) {}

    /**
     * @brief Default constructor for Node. Without any given arguments to initialize.
     */
    Node() {}

    /**
     * @brief Overloads the equality operator for comparing two Node objects in the unordered map CLOSED_LIST.
     *
     * @param n The Node object to compare with.
     * @return True if the x and y coordinates of the two Node objects are equal, false otherwise.
     */
    bool operator==(const Node& n) const { return n.x == x && n.y == y; }
};

/**
 * @brief Less than operator for priority queue. A struct to compare the f values of two nodes.
 *  Higher f-values have lower priority.
 */
struct CompareFValues {
    bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) { return n1->f > n2->f; }
};

/**
 * =============================================================================================
 *
 *                                          Typedefs
 *
 * =============================================================================================
 *
 * @brief A shared pointer to a Node object.
 */
typedef std::shared_ptr<Node> Nodeptr;

/**
 * @brief A custom closed list of type unordered_map to store visited nodes.
 */
typedef std::unordered_map<int, Nodeptr> CLOSED_LIST;

/**
 * @brief A custom open list of type priority_queue to store nodes to be visited.
 */
typedef std::priority_queue<Nodeptr, std::vector<Nodeptr>, CompareFValues> OPEN_LIST;

/**
 * =============================================================================================
 *
 *                                      Grid Planner Class
 *
 * =============================================================================================
 *
 * @brief A class for the grid planner.
 */
class GridPlanner {
   public:
    ros::Time plan_timeout_timer;  // planner's timeout timer

    double curr_yaw;  // current yaw angle of the robot, which is used for the orientation constraint

    /**
     * @brief Constructor creates a new GridPlanner.
     *
     * @param map_size The size of the map grid.
     * @param map_resolution The resolution of the map grid.
     * @param min_obs_cost The minimum obstacle cost, which is 100 for now.
     */
    GridPlanner(int map_size, float map_resolution, int min_obs_cost);

    /**
     * @brief Default constructor for GridPlanner. Without any arguments to initialize.
     */
    GridPlanner(){};

    /**
     * @brief Destructor destroys a GridPlanner.
     *
     * This destructor is responsible for cleaning up the map used by the GridPlanner object.
     * It calls the ResetMap() function to reset the stored map.
     */
    ~GridPlanner();

    /**
     * @brief Retrieves the map data.
     *
     * This function is used to retrieve the map data stored in the GridPlanner.
     *  The map data will be put into the vector map_data.
     *
     * @param map_data The vector to store the retrieved map.
     */
    void GetMap(std::vector<int8_t>& map_data);

    /**
     * @brief Prints the information of the GridPlanner object. Used for debugging.
     */
    void PrintInfo();

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
    void UpdateMap(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& origin_point);

    /**
     * @brief Updates the ground occupancy grid.
     */
    void UpdateMapBasedOnGround(const nav_msgs::OccupancyGrid& grid);

    /**
     * @brief Updates the dynamic waypoints from real-time input.
     */
    void UpdateWaypoints(const geometry_msgs::PoseArray& waypoints);

    /**
     * @brief Get the dynamic waypoints from the waypoints array in this planner.
     */
    void GetDynamicWaypoints(geometry_msgs::PoseArray& waypoints);

    /**
     * Global Plannomg with A* searching algorithm
     *
     * @param robot_pose The current pose of the robot.
     * @param target The target pose to reach.
     * @param plan The path to the target pose. Searching the global map to get this optimized path.
     * @return the number of the final waypoints in this searched path.
     */
    auto PlanWithAstar(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target,
                       geometry_msgs::PoseArray& plan) -> int;

    /**
     * @brief Global Planning with Theta* searching algorithm.
     *
     */
    auto PlanWithThetAstar(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target,
                           geometry_msgs::PoseArray& plan) -> int;

   private:
    /**
     * @brief The private fields of the global planner.
     *
     *  x_size: The x size of the map.
     *  y_size: The y size of the map.
     *  x_offset: The x offset of the map.
     *  y_offset: The y offset of the map. (transformation between the world and map coordinates)
     */
    int x_size, y_size, x_offset, y_offset;

    int max_steps;  // The maximum number of steps to search.

    /**
     *  map_resolution: The resolution of the map.
     *  obstacle_cost: The cost of an obstacle in the map.
     */
    float map_resolution, obstacle_cost;

    /**
     * @brief Searching directions in the 2D grid map.
     */
    int dX[8] = {LEFT, LEFT, LEFT, STAY, STAY, RIGHT, RIGHT, RIGHT};
    int dY[8] = {UP, STAY, DOWN, UP, DOWN, UP, STAY, DOWN};

    /**
     * @brief The start and goal nodes for the A* search algorithm. Based on the `Node` structure.
     */
    Node start_node, goal_node;

    /**
     * @brief The map used by the grid planner. Global Cost Map.
     */
    std::vector<int8_t, std::allocator<int8_t>> map;

    /**
     * @brief The map used by the grid planner. Dilated from the map.
     */
    std::vector<int8_t, std::allocator<int8_t>> configuration_space_map;

    /**
     * @brief The Dynamic Waypoints from real-time input
     */
    geometry_msgs::PoseArray dynamic_waypoints;

    /**
     * @brief The timer distance to reset the plan timer. This distance is used to find if the robot is still moving.
     */
    float timer_distance;

    /**
     * @brief The ground occupancy grid map. Formed with the ground cloud points.
     */
    nav_msgs::OccupancyGrid ground_occ_grid;

    /**
     * @brief Function to estimate the euclidean distance between two points. formula: sqrt(dx^2 + dy^2)
     */
    auto EstimateEuclideanDistance(int curr_x, int curr_y, int goal_x, int goal_y) -> float;

    /**
     * @brief Calculates the map index based on the given x and y coordinates.
     *  This is also the Key in Closed list.
     */
    int GetMapIndex(int, int);

    int GetGridMapIndex(int, int, int);

    /**
     * @brief Resets the map used by the grid planner.
     *  This function clears the map, removing all previously stored data.
     */
    void ResetMap();

    /**
     * @brief A function to initialize the map with the given map size.
     */
    void InitializeMap();

    /**
     * @brief A function to check if the given coordinates are within the map boundaries.
     */
    auto IsInMap(int x, int y) -> bool;

    /**
     * @brief A function to check if the given pose is within the map boundaries.
     */
    auto CheckPoseInMap(const geometry_msgs::Pose& pose, Node& node) -> bool;

    /**
     * @brief A function to set the A* cost of the node.
     */
    void SetAstarCost(Nodeptr& nodeptr, float g, float h);

    /**
     * @brief A function to calculate the angle between two poses.
     *
     *  The angle is calculated based on the orientation of the two poses. Formula: atan2(y2 - y1, x2 - x1).
     */
    auto CalculateAngle(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2) -> float;

    /**
     * @brief A function for theta* search algorithm. It decides if there is a straight free-obstacle line between two
     * nodes.
     */
    auto LineOfSight(const Nodeptr& start, const Nodeptr& end) -> bool;

    /**
     * @brief A function to convert the workspace to the configuration space. Dilate the map with opencv.
     */
    void ThickenObstacles(std::vector<int8_t, std::allocator<int8_t>>& map_data_to_thicken);

    /**
     * @brief A function to check if the robot can move to the given direction. This is our 2.5D searching constraint.
     *  Work in a way that filters out constrainted path in the process of searching expansion. 
     * 
     * This algorithm function could ensure the robot could get the plan with reasonable rotations in the path.
     */
    auto OrientationConstraint(int direction, int yaw) -> bool;

    /**
     * @brief A function to wrap the angle to [-PI, PI). Clip the angle to a proper range.
     *
     *  Mod the angle by 2 * PI, we have [0, 2 * M_PI). Then, if the angle is greater than PI, we subtract 2 * PI. If
     * the angle is less than -PI, we add 2 * PI.
     *
     * @param angle The angle to wrap.
     * @return The wrapped angle.
     */
    inline float Wrap2PI(float angle) {
        angle = fmod(angle, 2 * M_PI);
        angle += angle >= M_PI ? -2 * M_PI : (angle < -M_PI ? 2 * M_PI : 0);

        return angle;
    }
};

#endif
