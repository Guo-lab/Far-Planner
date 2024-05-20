#ifndef _PLANNER_ROSN_H_
#define _PLANNER_ROS_H_

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>

#include "grid_planner.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "mmpug_msgs/RobotPlan.h"

#include <string>
#include <iostream> 
#include <algorithm>
#include <vector>
#include <stdlib.h>

class PlannerNode {
    public:
        PlannerNode();
        ~PlannerNode();
        void Run();
        // The Buffer is used to call LookupTransform function
        // do the transformation between the global frame and the frame receiving occupancy grid msg
        // tfBuffer is a pointer to a tf2_ros::Buffer object
        tf2_ros::Buffer *tfBuffer;

    private:
        // ROS Node Handlers
        ros::NodeHandle nh;
        ros::NodeHandle p_nh = ros::NodeHandle("~");
        
        // ROS Subscribers
        ros::Subscriber occupancy_grid_sub;
        ros::Subscriber waypoints_sub;
        ros::Subscriber current_pose_sub;

        // ROS Publishers
        ros::Publisher plan_publisher;
        ros::Publisher stats_pub;
        
        // void OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr&);
        void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void plannerReqHandler(const geometry_msgs::PoseArray::ConstPtr&);
        void plannerCurrPoseHandler(const nav_msgs::Odometry::ConstPtr&);
        
        float ros_rate;
        std::string costmap_topic;
        std::string global_frame_id;
        std::string planner_service_topic;
        bool publish_global_grid;

        float map_resolution;
        int map_size;
        int min_obstacle_cost;

        GridPlanner planner;
        nav_msgs::OccupancyGrid new_occ_grid;
        /* Odometry data -> PoseWithCovariance -> Pose */
        nav_msgs::Odometry current_odom;
        
        bool initialized_map;
        bool init_robot_pose;
        bool init_targets;
};
#endif
