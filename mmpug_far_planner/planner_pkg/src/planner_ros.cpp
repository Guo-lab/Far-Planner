/**
    * @file planner_ros.cpp
    * @brief Planner ROS Node Implementation 
    * @ref https://github.com/mmpug-archive/mmpug_far_planner/tree/ps/new_planner
 */
#include "planner_ros.h"
#include <fstream>


/**
 * PlannerNode constructor. Base class: nh (Node Handle)
*/
PlannerNode::PlannerNode():nh(){
    initialized_map = false;
    
    p_nh.getParam("ros_rate", ros_rate);
    
    p_nh.getParam("global_frame_id", global_frame_id);
    p_nh.getParam("planner_service_topic", planner_service_topic);
    p_nh.getParam("costmap_topic", costmap_topic);
    
    p_nh.getParam("scanVoxelSize", map_resolution);
    p_nh.getParam("plannerMapSize", map_size);
    p_nh.getParam("occupied_min_value", min_obstacle_cost);
    p_nh.getParam("publish_global_grid", publish_global_grid);

    ROS_INFO_STREAM("Global Frame Id: " << global_frame_id);
    ROS_INFO_STREAM("Planner Service Topic: " << planner_service_topic);
    ROS_INFO_STREAM("Costmap Topic: " << costmap_topic);
    ROS_INFO_STREAM("Map Resolution: " << map_resolution);
    ROS_INFO_STREAM("Map Size: " << map_size);
    ROS_INFO_STREAM("Min Obstacle Cost: " << min_obstacle_cost);
    ROS_INFO_STREAM("Publish Global Grid: " << publish_global_grid);

    planner = GridPlanner(map_size, map_resolution, min_obstacle_cost);
}
/* PlannerNode destructor */
PlannerNode::~PlannerNode(){}



/**
 * Planner Node Run Function
 *  First, listen for occupancy grid data on the topic specified by costmap_topic. 
 *      and advertise occupancy grid data on the topic "/final_occ_grid".
 *  Then, init transform listener.
 *  Then, handle planning requests on the topic planner_service_topic. The service server is configured to call the plannerReqHandler method of the PlannerNode class when a request is received.
*/
void PlannerNode::Run() { 
ROS_INFO("Global Planner Running");

    // Create ROS Subs
    // occupancy_grid_sub = nh.subscribe(costmap_topic, 1, &PlannerNode::OccupancyGridHandler, this);
    costmap_sub = nh.subscribe("cmu_rc1/local_mapping_lidar_node/voxel_grid/obstacle_map", 1, &PlannerNode::CostmapCallback, this);
    waypoints_sub = nh.subscribe("cmu_rc1/command_interface/waypoint", 10, &PlannerNode::plannerReqHandler, this);
    current_pose_sub = nh.subscribe("cmu_rc1/odom_to_base_link", 1, &PlannerNode::plannerCurrPoseHandler, this);
    // Create ROS Pubs
    plan_publisher = nh.advertise<geometry_msgs::PoseArray>("cmu_rc1/mux/goal_input", 1, true);
    global_cost_map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("cmu_rc1/final_occ_grid", 1, true);
    
    tf2_ros::Buffer tfBuffer_;
    tfBuffer = &tfBuffer_;

    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()) {
        // Checks if OccupancyGrid has been initialized and if publishing_global is enabled
        if(initialized_map && publish_global_grid) {
            // Retrieve the occupancy grid data from the planner object, 
            // updates new_occ_grid map and its metadata, 
            /* <nav_msgs::OccupancyGrid> new_occ_grid
                Header header 
                MapMetaData info
                # The map data, in row-major order, starting with (0,0).  Occupancy
                # probabilities are in the range [0,100].  Unknown is -1.
                int8[] data (std::vector<int8_t>& map_data)
            */
            planner.getMap(new_occ_grid.data);
            new_occ_grid.header.frame_id = global_frame_id;
            new_occ_grid.info.resolution = map_resolution;
            new_occ_grid.info.height = map_size;
            new_occ_grid.info.width = map_size;
            new_occ_grid.info.origin.position.x = - map_resolution * map_size/2;
            new_occ_grid.info.origin.position.y = - map_resolution * map_size/2;
            new_occ_grid.header.stamp = ros::Time::now();

            // Publish the occupancy grid data
            global_cost_map_publisher.publish(new_occ_grid);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * PlannerNode OccupancyGridHandler
 * @param msg: mmpug_msgs::MMPUGOccupancyMsg::ConstPtr&
*/
// void PlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){
// ROS_INFO("Updating MAP For Planner ");
//     geometry_msgs::TransformStamped tfStamp = tfBuffer->lookupTransform(global_frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(0.25));
    
//     // Compute 2 corners to iterate through the vector in global
//     // Assign msg->info.origin.position to p1 and init a p1t (the transformed origin got by doTransform())
//     geometry_msgs::Point p1 = msg->info.origin.position;
//     geometry_msgs::Point p1t;
//     tf2::doTransform(p1, p1t, tfStamp); // No need in new architecture

//     // Update the Map of Grid Planner (const nav_msgs::OccupancyGrid& grid, geometry_msgs::Point& corner1);
//     planner.updateMap(*msg, p1t);
//     initialized_map = true;
// }
void PlannerNode::CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
ROS_INFO("Updating Cost MAP For Planner");

    // geometry_msgs::TransformStamped tfStamp = tfBuffer->lookupTransform(global_frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(0.25));
    // geometry_msgs::Point p1 = msg->info.origin.position;
    // geometry_msgs::Point p1t;
    // tf2::doTransform(p1, p1t, tfStamp); // No need in new architecture

    // auto m_occupancyGrid = *msg;
    // m_costmap = Costmap(m_occupancyGrid.info.origin.position.x,
    //                     m_occupancyGrid.info.origin.position.y,
    //                     m_occupancyGrid.info.resolution,
    //                     m_occupancyGrid.info.width,
    //                     m_occupancyGrid.info.height);
    // for (auto &cell : m_occupancyGrid.data)
    // {
    //     m_costmap.data.push_back(static_cast<int>(cell));
    // }

    geometry_msgs::Point origin_point = msg->info.origin.position;
    planner.updateMap(*msg, origin_point);
    ROS_INFO_STREAM("Map updated: " << msg->info.resolution << " " << msg->info.width << " " << msg->info.height);
    ROS_INFO_STREAM("Origin: " << msg->info.origin.position.x << " " << msg->info.origin.position.y);
    initialized_map = true;
}

/**
 * PlannerNode plannerReqHandler
*/
void PlannerNode::plannerReqHandler( const geometry_msgs::PoseArray::ConstPtr& req ) { // former PoseStamped::ConstPtr& req
    geometry_msgs::PoseArray plan;
    
    for (auto each_pose : req->poses) {
        ROS_INFO_STREAM("Received pose: " << each_pose);
    }
    // geometry_msgs::Pose robot_pose = req->poses[0];
    geometry_msgs::Pose robot_pose = current_odom.pose.pose;
    ROS_INFO_STREAM("Robot pose: " << robot_pose);

    // naivePlanner based on request, computed a plan for the robot's path.
    ros::Time s1 = ros::Time::now();
    int planLength = planner.naivePlanner(robot_pose, req->poses[req->poses.size() - 1], plan);
    ros::Time s2 = ros::Time::now();
    ros::Duration d = s2 - s1;
    ROS_INFO_STREAM("Planning routine complete. Time taken: " << d.toSec());
    
    ROS_INFO_STREAM("Plan length: " << planLength);
    ROS_INFO_STREAM("Frame Id: " << plan.header.frame_id);
    for (auto each_pose : plan.poses) {
        ROS_INFO_STREAM("Plan pose: " << each_pose);
    }

    plan.header.frame_id = "global";
    plan.header.stamp = ros::Time::now();
    if (planLength > 0) {
        ROS_INFO("Plan successful");
        plan_publisher.publish(plan);

        // std::vector<int8_t> cost_occupancy_map_log;
        // planner.getMap(cost_occupancy_map_log);
        // std::ofstream log_file("CostMap.log");
        // log_file << "\nCost occupancy map log: ";
        // for (auto each_pixel : cost_occupancy_map_log) {
        //     log_file << each_pixel << " ";
        // }
        // log_file.close();

    } else {
        ROS_INFO("Plan failed");
    }
}

void PlannerNode::plannerCurrPoseHandler(const nav_msgs::Odometry::ConstPtr &msg) {
    current_odom = *msg;
}


/**
 * Main function for the planner_ros. 
 * ros init with ROS Node: Landing Planner Node and run it.
*/
int main(int argc, char** argv) {
    ROS_INFO("Starting Landing Planner Node ");
    ros::init(argc, argv, "Landing Planner Node");
    PlannerNode node;
    node.Run();
    return 0;
}