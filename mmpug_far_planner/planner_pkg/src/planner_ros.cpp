/**
 * @file planner_ros.cpp
 * @brief Planner ROS Node Implementation
 * @ref https://github.com/mmpug-archive/mmpug_far_planner/tree/ps/new_planner
 * @author Siqi. Edward, Guo
 * @version 1.0
 */
#include "planner_ros.h"

// #include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
// #include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h> /** for converting PointCloud2 to PCL data types */

// ros::Time timer;
geometry_msgs::PoseArray verify_waypoints;

/**
 * PlannerNode Constructor.
 */
PlannerNode::PlannerNode() : node_handler() {
    initialized_map = false;
    planning = false;

    private_node_handler.getParam("ros_rate", ros_rate);
    private_node_handler.getParam("global_frame_id", global_frame_id);

    private_node_handler.getParam("scanVoxelSize", map_resolution);
    private_node_handler.getParam("plannerMapSize", map_size);
    private_node_handler.getParam("occupied_min_value", min_obstacle_cost);

    planner = GridPlanner(map_size, map_resolution, min_obstacle_cost);
}
/**
 * PlannerNode Destructor.
 */
PlannerNode::~PlannerNode() {}

/**
 * Planner Node Run Function
 *  This function creates the subscribers and publishers, and runs the planner node.
 *  It subscribes to the local cost map data from local mapping lidar, the manual-set waypoints from the command
 * interface, and the current pose of the robot from the odometry data. It also publishes the global cost map data and
 * the A*-path (to Rviz). The global cost map data is retrieved from the planner object. The OccupancyGrid message is
 * updated with the new map data and metadata and published.
 */
void PlannerNode::Run() {
    ROS_INFO("Global Planner Running");

    costmap_sub = node_handler.subscribe("cmu_rc1/local_mapping_lidar_node/voxel_grid/obstacle_map", 1,
                                         &PlannerNode::CostmapCallback, this);
    waypoints_sub =
        node_handler.subscribe("cmu_rc1/command_interface/waypoint", 10, &PlannerNode::HandleWaypointsRequest, this);
    current_pose_sub = node_handler.subscribe("cmu_rc1/odom_to_base_link", 1, &PlannerNode::HandleCurrentPose, this);
    
    ground_cloud_sub = node_handler.subscribe("cmu_rc1/local_mapping_lidar_node/voxel_grid/ground_pointcloud", 1,
                                              &PlannerNode::GroundCloudCallback, this);
    obstacle_cloud_sub = node_handler.subscribe("cmu_rc1/local_mapping_lidar_node/voxel_grid/obstacle_pointcloud", 1,
                                                &PlannerNode::ObstacleCloudCallback, this);

    plan_publisher = node_handler.advertise<geometry_msgs::PoseArray>("cmu_rc1/mux/goal_input", 1, true);
    global_cost_map_publisher = node_handler.advertise<nav_msgs::OccupancyGrid>("cmu_rc1/final_occ_grid", 1, true);
    a_star_path_to_goal_publisher = node_handler.advertise<nav_msgs::Path>("cmu_rc1/a_star_path_to_goal", 1, true);
    theta_star_path_to_goal_publisher =
        node_handler.advertise<nav_msgs::Path>("cmu_rc1/theta_star_path_to_goal", 1, true);

    ros::spinOnce();
    ros::Rate rate(ros_rate);

    while (ros::ok()) {
        if (initialized_map) {
            planner.GetMap(new_occ_grid.data);
            ReloadGridMetadata();
            global_cost_map_publisher.publish(new_occ_grid);
        }

        if (planning) {
            planning = ReplanTillGoal();
            if (planning && ros::Time::now() - planner.plan_timer > ros::Duration(10)) {
                ROS_INFO("TIMEOUT. NO PATH FOUND.");
                planning = false;
                planner.plan_timer = ros::Time::now();
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

auto PlannerNode::ReplanTillGoal() -> bool {
    geometry_msgs::PoseArray waypoints;
    planner.GetDynamicWaypoints(waypoints);

    if (waypoints.poses.size() == 0) {
        ROS_INFO("NO WAYPOINTS TO PLAN, PLAN TERMINATED");
        planner.plan_timer = ros::Time::now();
        return false;
    }

    geometry_msgs::PoseArray a_star_plan;
    geometry_msgs::PoseArray theta_star_plan;

    geometry_msgs::Pose robot_pose = current_odom.pose.pose;

    // nav_msgs::Path path_to_goal;
    nav_msgs::Path a_star_path_to_goal;
    nav_msgs::Path theta_star_path_to_goal;

    ros::Time s1 = ros::Time::now();

    int plan_length_from_current_pose_to_first_waypoint =
        planner.PlanWithAstar(robot_pose, waypoints.poses[0], a_star_plan);
    plan_length_from_current_pose_to_first_waypoint =
        planner.PlanWithThetAstar(robot_pose, waypoints.poses[0], theta_star_plan);

    if (plan_length_from_current_pose_to_first_waypoint <= 0) {
        if (plan_length_from_current_pose_to_first_waypoint < 0) {
            ROS_WARN("PLAN FAILED. NO PATH FOUND.");
            planner.plan_timer = ros::Time::now();
            return false;
        }

        ROS_INFO("REACH ONE WAYPOINT");
        geometry_msgs::PoseArray tmp_pose_array;
        planner.GetDynamicWaypoints(tmp_pose_array);

        int waypoints_size = tmp_pose_array.poses.size();
        tmp_pose_array.poses.erase(tmp_pose_array.poses.begin());
        ROS_ASSERT(waypoints_size - 1 == tmp_pose_array.poses.size());
        planner.UpdateWaypoints(tmp_pose_array);

        planner.plan_timer = ros::Time::now();
        return true;
    }

    ros::Time s2 = ros::Time::now();
    ros::Duration d = s2 - s1;
    ROS_INFO_STREAM("PLANNING ROUTINE COMPLETE. TIME TAKEN: " << d.toSec());

    a_star_plan.header.frame_id = global_frame_id;
    a_star_plan.header.stamp = ros::Time::now();
    theta_star_plan.header.frame_id = global_frame_id;
    theta_star_plan.header.stamp = ros::Time::now();

    for (int i = 1; i < waypoints.poses.size(); i++) {
        a_star_plan.poses.push_back(waypoints.poses[i]);
        theta_star_plan.poses.push_back(waypoints.poses[i]);
    }

    // plan_publisher.publish(a_star_plan);
    plan_publisher.publish(theta_star_plan);

    ReloadPathToGoal(a_star_path_to_goal, a_star_plan);
    ReloadPathToGoal(theta_star_path_to_goal, theta_star_plan);

    a_star_path_to_goal_publisher.publish(a_star_path_to_goal);
    theta_star_path_to_goal_publisher.publish(theta_star_path_to_goal);
    return true;
}

/**
 * @brief Callback function for updating the cost map used by the planner.
 *
 * This function is called when a new occupancy grid message is received.
 * It updates the cost map used by the planner with the new map data and origin point in local planner's map.
 *
 * @param msg The received occupancy grid message.
 */
void PlannerNode::CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // ROS_INFO("UPDATING COST MAP FOR PLANNER...");
    geometry_msgs::Point origin_point = msg->info.origin.position;
    planner.UpdateMap(*msg, origin_point);
    initialized_map = true;
}

void PlannerNode::GroundCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // ROS_INFO("UPDATING GROUND CLOUD FOR PLANNER...");
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    const float ground_z = 0.0;

    planner.ground_cloud.clear();
    for (const auto& each_point : cloud->points) {
        auto point_ptr = std::make_shared<geometry_msgs::Point>();
        point_ptr->x = each_point.x;
        point_ptr->y = each_point.y;
        point_ptr->z = ground_z;
        planner.ground_cloud.push_back(point_ptr);
        // ROS_INFO("Point: (%f, %f)", x, y);
    }
    // ROS_INFO("Cloud Size: %ld", cloud->size());
    // ROS_INFO("Ground Cloud Size: %ld", planner.ground_cloud.size());
}

void PlannerNode::ObstacleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // ROS_INFO("UPDATING OBSTACLE CLOUD FOR PLANNER...");

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    planner.obstacle_cloud.clear();
    for (const auto& each_point : cloud->points) {
        auto point_ptr = std::make_shared<geometry_msgs::Point>();
        point_ptr->x = each_point.x;
        point_ptr->y = each_point.y;
        point_ptr->z = each_point.z;
        planner.obstacle_cloud.push_back(point_ptr);
    }
    ROS_INFO("Cloud Size: %ld", cloud->size());
    ROS_INFO("Obstacle Cloud Size: %ld", planner.obstacle_cloud.size());
}




/**
 * PlannerNode HandleWaypointsRequest
 *  Receiving a series of waypoints from the command interface.
 *  The planner will plan a path from the current pose to the next waypoint. This path will be store into a temporary
 * PoseArray. The PoseArrya will be appended to the final point by many pathes.
 */
void PlannerNode::HandleWaypointsRequest(const geometry_msgs::PoseArray::ConstPtr& req) {
    ROS_INFO("RECEIVED WAYPOINTS REQUEST...");
    planner.UpdateWaypoints(*req);
    verify_waypoints = *req;
    planning = true;
    planner.plan_timer = ros::Time::now();
}

/**
 * @brief Callback function for handling the current pose of the planner.
 *
 * This function is called whenever a new odometry message is received.
 * It updates the current_odom member variable with the received message.
 *
 * @param msg The odometry message containing the current pose information.
 */
void PlannerNode::HandleCurrentPose(const nav_msgs::Odometry::ConstPtr& msg) { current_odom = *msg; }

/**
 * @brief Update the metadata of the global cost map to publish.
 *  Including - Header header - MapMetaData info.
 */
void PlannerNode::ReloadGridMetadata() {
    new_occ_grid.header.frame_id = global_frame_id;
    new_occ_grid.info.resolution = map_resolution;
    new_occ_grid.info.height = map_size;
    new_occ_grid.info.width = map_size;
    new_occ_grid.info.origin.position.x = -map_resolution * map_size / 2;
    new_occ_grid.info.origin.position.y = -map_resolution * map_size / 2;
    new_occ_grid.header.stamp = ros::Time::now();
}

/**
 * @brief Update the path to the goal position for publisher.
 *  Just copy the pose array from the planner's final decision to the path message.
 * @param path The path message to update.
 * @param plan The pose array from the planner's final decision.
 */
void PlannerNode::ReloadPathToGoal(nav_msgs::Path& path, const geometry_msgs::PoseArray& plan) {
    path.header = plan.header;

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header = plan.header;
    robot_pose.pose = current_odom.pose.pose;
    path.poses.push_back(robot_pose);

    for (auto pose : plan.poses) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = plan.header;
        pose_stamped.pose = pose;
        path.poses.push_back(pose_stamped);
    }
}

/**
 * @brief The main function of the planner node.
 *
 * This function initializes the ROS node, creates an instance of the PlannerNode class,
 * and calls the Run() function to start the planner node.
 *
 * @param argc The number of command-line arguments.
 * @param argv An array of command-line arguments.
 * @return int Returns 0 upon successful execution.
 */
int main(int argc, char** argv) {
    ROS_INFO("STARTING GLOBAL PLANNER NODE...");
    ros::init(argc, argv, "Landing Global Planner Node");
    PlannerNode node;
    node.Run();
    return 0;
}
