/**
 * @file planner_ros.cpp
 * @brief Planner ROS Node Implementation
 * @ref https://github.com/mmpug-archive/mmpug_far_planner/tree/ps/new_planner
 * @author Siqi. Edward, Guo
 * @version 1.0
 */
#include "planner_ros.h"


/**
 * PlannerNode constructor. 
 */
PlannerNode::PlannerNode() : node_handler() {
  initialized_map = false;

  private_node_handler.getParam("ros_rate", ros_rate);

  private_node_handler.getParam("global_frame_id", global_frame_id);
  private_node_handler.getParam("planner_service_topic", planner_service_topic);
  private_node_handler.getParam("costmap_topic", costmap_topic);

  private_node_handler.getParam("scanVoxelSize", map_resolution);
  private_node_handler.getParam("plannerMapSize", map_size);
  private_node_handler.getParam("occupied_min_value", min_obstacle_cost);
  private_node_handler.getParam("publish_global_grid", publish_global_grid);

  planner = GridPlanner(map_size, map_resolution, min_obstacle_cost);
}
PlannerNode::~PlannerNode() {}

/**
 * Planner Node Run Function
 *  First, listen for occupancy grid data on the topic specified by
 * costmap_topic. and advertise occupancy grid data on the topic
 * "/final_occ_grid". Then, init transform listener. Then, handle planning
 * requests on the topic planner_service_topic. The service server is configured
 * to call the plannerReqHandler method of the PlannerNode class when a request
 * is received.
 */
void PlannerNode::Run() {
  ROS_INFO("Global Planner Running");

  costmap_sub = node_handler.subscribe("cmu_rc1/local_mapping_lidar_node/voxel_grid/obstacle_map", 1, &PlannerNode::CostmapCallback, this);
  waypoints_sub = node_handler.subscribe("cmu_rc1/command_interface/waypoint", 10, &PlannerNode::plannerReqHandler, this);
  current_pose_sub = node_handler.subscribe("cmu_rc1/odom_to_base_link", 1, &PlannerNode::plannerCurrPoseHandler, this);

  plan_publisher = node_handler.advertise<geometry_msgs::PoseArray>("cmu_rc1/mux/goal_input", 1, true);
  global_cost_map_publisher = node_handler.advertise<nav_msgs::OccupancyGrid>("cmu_rc1/final_occ_grid", 1, true);
  path_to_goal_publisher = node_handler.advertise<nav_msgs::Path>("cmu_rc1/path_to_goal", 1, true);

  ros::spinOnce();
  ros::Rate rate(ros_rate);

  while (ros::ok()) {
    if (initialized_map && publish_global_grid) {
      // Retrieves the occupancy grid data from the planner object, updates new_occ_grid map and its metadata,

      planner.getMap(new_occ_grid.data);
      new_occ_grid.header.frame_id = global_frame_id;
      new_occ_grid.info.resolution = map_resolution;
      new_occ_grid.info.height = map_size;
      new_occ_grid.info.width = map_size;
      new_occ_grid.info.origin.position.x = -map_resolution * map_size / 2;
      new_occ_grid.info.origin.position.y = -map_resolution * map_size / 2;
      new_occ_grid.header.stamp = ros::Time::now();

      // Publish the occupancy grid data
      global_cost_map_publisher.publish(new_occ_grid);
    }
    ros::spinOnce();
    rate.sleep();
  }
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
  ROS_INFO("Updating Cost MAP For Planner");
  geometry_msgs::Point origin_point = msg->info.origin.position;
  planner.updateMap(*msg, origin_point);
  initialized_map = true;
}

/**
 * PlannerNode plannerReqHandler
 */
void PlannerNode::plannerReqHandler(const geometry_msgs::PoseArray::ConstPtr& req) {
  geometry_msgs::PoseArray plan;
  geometry_msgs::Pose robot_pose = current_odom.pose.pose;

  ros::Time s1 = ros::Time::now();
  int planLength = planner.naivePlanner(robot_pose, req->poses[req->poses.size() - 1], plan);
  ros::Time s2 = ros::Time::now();
  ros::Duration d = s2 - s1;
  ROS_INFO_STREAM("Planning routine complete. Time taken: " << d.toSec());

  plan.header.frame_id = "global";
  plan.header.stamp = ros::Time::now();
  if (planLength > 0) {
    ROS_INFO("Plan successful");
    plan_publisher.publish(plan);

    nav_msgs::Path path_to_goal;
    path_to_goal.header = plan.header;

    for (auto pose : plan.poses) {
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header = plan.header;
      pose_stamped.pose = pose;
      path_to_goal.poses.push_back(pose_stamped);
    }
    path_to_goal_publisher.publish(path_to_goal);

  } else { ROS_WARN("Plan failed"); }
}

/**
 * @brief Callback function for handling the current pose of the planner.
 * 
 * This function is called whenever a new odometry message is received.
 * It updates the current_odom member variable with the received message.
 * 
 * @param msg The odometry message containing the current pose information.
 */
void PlannerNode::plannerCurrPoseHandler(const nav_msgs::Odometry::ConstPtr& msg) {
  current_odom = *msg;
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
  ROS_INFO("Starting Global Planner Node");
  ros::init(argc, argv, "Landing Global Planner Node");
  PlannerNode node;
  node.Run();
  return 0;
}

