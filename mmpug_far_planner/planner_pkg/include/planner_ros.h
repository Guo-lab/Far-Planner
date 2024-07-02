#ifndef _PLANNER_ROSN_H_
#define _PLANNER_ROS_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>

/**For Rviz visualization-specific data */
#include <visualization_msgs/Marker.h>

/**In the old architecture, it is required to use tf2 library
 *  to handle coordinate transformations between different coordinate frames.
 * The transform listener is used to listen to coordinate transformations broadcasted on the ROS network.
 * In the former implementation, lookupTransform() is also used to query the buffer the latest
 *  transform between global frame and local frame.
 * However, in the new architecture, the frame has been consistent, thus tf2 is no more needed.
 */

#include <stdlib.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "grid_planner.h"

/**
 * @brief Class representing the Global (FAR) Planner Node.
 *
 * This class handles the global planning functionality of the robot.
 */
class PlannerNode {
   public:
    /**
     * @brief Default constructor for PlannerNode class. Base class: node_handler (Node Handle)
     */
    PlannerNode();

    /**
     * @brief Destructor for PlannerNode class.
     */
    ~PlannerNode();

    /**
     * @brief Runs the planner node.
     */
    void Run();

   private:
    /**
     * @brief The handler to this ROS node, a way to interact with the ROS system.
     */
    ros::NodeHandle node_handler;
    ros::NodeHandle private_node_handler = ros::NodeHandle("~");

    /* ROS Subscribers and Publishers. */

    /**
     * @brief A subscriber object that receives messages from one ROS topic.
     *  HandleWaypointsRequest is the callback function, used to get the local planner's
     *  waypoints from the command interface.
     */
    ros::Subscriber waypoints_sub;

    /**
     * @brief A subscriber object subscribe the topic <>.
     *  HandleCurrentPose is the callback function, used to get the current pose of the robot.
     */
    ros::Subscriber current_pose_sub;

    /**
     * @brief A subscriber object that receives messages from one ROS topic.
     *  CostmapCallback is the callback function, used to get the occupancy grid data
     *  from the local planner's local cost map.
     */
    ros::Subscriber costmap_sub;
    ros::Subscriber ground_sub;
    // ros::Subscriber ground_cloud_sub;
    // ros::Subscriber obstacle_cloud_sub;

    /**
     * @brief A ROS publisher for publishing the global planner's plan messages.
     */
    ros::Publisher plan_publisher;

    /**
     * @brief A ROS publisher for publishing the global cost map.
     *  This will be used to visualize the global cost map in Rviz.
     */
    ros::Publisher global_cost_map_publisher;

    /**
     * @brief A ROS publisher for publishing the path from the current position to the goal.
     *  This will be used to visualize the path in Rviz.
     */
    ros::Publisher a_star_path_to_goal_publisher;
    ros::Publisher theta_star_path_to_goal_publisher;

    /**
     * @brief Callback function for the costmap subscriber, updating the cost map used by the planner.
     *
     * This function is called when a new occupancy grid message is received.
     * It updates the cost map used by the planner with the new map data and origin point in local planner's map.
     *
     * @param msg The received occupancy grid message, aka, costmap message.
     */
    void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    void GroundCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    // void GroundCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    // void ObstacleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    /**
     * @brief Callback function for the planner request subscriber.
     * @param msg The received planner request message.
     */
    void HandleWaypointsRequest(const geometry_msgs::PoseArray::ConstPtr&);

    /**
     * @brief Callback function for the current pose subscriber.
     * @param msg The received current pose message.
     */
    void HandleCurrentPose(const nav_msgs::Odometry::ConstPtr&);

    /**
     * @brief Update the metadata of the global cost map to publish.
     */
    void ReloadGridMetadata();

    /**
     * @brief Update the path to the goal position for publisher.
     */
    void ReloadPathToGoal(nav_msgs::Path& path, const geometry_msgs::PoseArray& plan);

    auto ReplanTillGoal() -> bool;

    /**
     * @brief The ROS rate for the planner node.
     */
    float ros_rate;
    /**
     * @brief The global frame ID. Default is "global".
     */
    std::string global_frame_id;
    /**
     * @brief The flag to show whether the map has been initialized.
     *  Default is false. Set to true when the cost map has been updated once.
     */
    bool initialized_map;
    bool planning;

    /**
     * @brief The map resolution.
     */
    float map_resolution;
    /**
     * @brief The map size. Default (x_size, y_size) := (1000, 1000)
     */
    int map_size;
    /**
     * @brief The minimum obstacle cost.
     *  Default is 100, which is also the occupancy probability to visualize the map cell.
     */
    int min_obstacle_cost;

    /**
     * @brief The GridPlanner class represents a planner for grid-based path planning.
     *  Defined in GRID_PLANNER_H.
     */
    GridPlanner planner;

    /**
     * @brief Represents an occupancy grid map, which is the global cost map maintained by the FAR Planner
     *
     * <nav_msgs::OccupancyGrid> new_occ_grid has the following fields:
     *  - Header header
     *  - MapMetaData info
     *  - int8[] data (std::vector<int8_t>& map_data) # in row-major order, starting with (0,0).
     * Occupancy probabilities are in the range [0,100].  Unknown is -1.
     */
    nav_msgs::OccupancyGrid new_occ_grid;

    /**
     * @brief Represents the current odometry information.
     *
     * Here, the current robot position can be retrieved from this structure.
     * Odometry data -> PoseWithCovariance -> Pose
     */
    nav_msgs::Odometry current_odom;
};

#endif
