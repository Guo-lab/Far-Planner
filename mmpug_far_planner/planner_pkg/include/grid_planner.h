#ifndef GRID_PLANNER_H
#define GRID_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>

#include <queue>
#include <vector>
#include <unordered_map>
#include <string>
#include <iostream> 
#include <algorithm>
#include <cmath>
#include <stdlib.h>

#ifndef MIN
#define MIN(A, B) ((A) < (B) ? (A) : (B))
#endif


struct Node {
    int key, time;
    int x, y, theta;
    float g, h, f;
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
    bool operator==(const Node &n) const { return n.x == x && n.y == y; }
};

// Less than operator for priority queue
struct CompareFValues {
    auto operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) -> bool { return n1->f > n2->f; }
};


typedef std::shared_ptr<Node> Nodeptr;

//Defining a custom closed list of type unordered_map to Store visited nodes.
typedef std::unordered_map<int, Nodeptr> CLOSED_LIST; 

//Defining an open list of type priority queue
typedef std::priority_queue<Nodeptr, std::vector<Nodeptr>, CompareFValues> OPEN_LIST;



class GridPlanner {
    public:
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
        GridPlanner() {};

        /**
         * @brief Destructor destroys a GridPlanner.
         * 
         * This destructor is responsible for cleaning up the map used by the GridPlanner object.
         * It calls the resetMap() function to reset the stored map.
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
        void getMap(std::vector<int8_t>& map_data);

        /**
         * @brief Prints the information of the GridPlanner object.
         */
        void printInfo();

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
        void updateMap(const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& origin_point);

        /** Global Plannomg with A* searching algorithm
         * 
         */
        auto naivePlanner(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target, geometry_msgs::PoseArray& plan) -> int;

    private:
        int x_size, y_size, x_offset, y_offset;
        int max_steps;        
        float map_resolution;
        float obstacle_cost;
        
        int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
        int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

        Node start_node, goal_node;

        std::vector<int8_t, std::allocator<int8_t>> map;

        auto estimateEuclideanDistance(int curr_x, int curr_y, int goal_x, int goal_y) -> float;

        int getMapIndex(int, int);
        void resetMap();
        void initializeMap();
        auto isInMap(int x, int y) -> bool;

        inline float wrap2PI(float angle) {
            angle = fmod(angle, 2 * M_PI);
            if (angle >= M_PI) {
                angle -= 2 * M_PI;
            }
            if(angle < -M_PI) {
                angle += 2 * M_PI;
            }
            return angle;
        }  

};
#endif
