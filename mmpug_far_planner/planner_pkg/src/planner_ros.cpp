#include "planner_ros.h"


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
    occupancy_grid_sub = nh.subscribe(costmap_topic, 1, &PlannerNode::OccupancyGridHandler, this);
    waypoints_sub = nh.subscribe("cmu_rc1/command_interface/waypoint", 10, &PlannerNode::plannerReqHandler, this);
    current_pose_sub = nh.subscribe("cmu_rc1/odom_to_base_link", 1, &PlannerNode::plannerCurrPoseHandler, this);
    stats_pub = nh.advertise<nav_msgs::OccupancyGrid>("/final_occ_grid", 1, true);
    plan_publisher = nh.advertise<geometry_msgs::PoseArray>("cmu_rc1/mux/goal_input", 1, true);
    
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_(tfBuffer_);
    tfBuffer = &tfBuffer_;
    tfListener = &tfListener_;
    
    // ros::ServiceServer service = nh.advertiseService(planner_service_topic, &PlannerNode::plannerReqHandler, this);

    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    while(ros::ok()){ // when ROS is still running

        // Checks if the map has been initialized and if publishing of the global grid is enabled
        if(initialized_map && publish_global_grid){

            // Retrieve the occupancy grid data from the planner object, updates metadata, 
            // publish it using the stats_pub publisher.
            // std::vector<int8_t>& map_data: nav_msgs::OccupancyGrid new_occ_grid 
            planner.getMap(new_occ_grid.data);
            new_occ_grid.header.frame_id = global_frame_id;
            new_occ_grid.info.resolution = map_resolution;
            new_occ_grid.info.height = map_size;
            new_occ_grid.info.width = map_size;
            new_occ_grid.info.origin.position.x = - map_resolution * map_size/2;
            new_occ_grid.info.origin.position.y = - map_resolution * map_size/2;
            new_occ_grid.header.stamp = ros::Time::now();

            // stats_pub.publish(new_occ_grid);
        }

        ros::spinOnce();
        rate.sleep();
    }
}

/**
 * PlannerNode OccupancyGridHandler
 * @param msg: mmpug_msgs::MMPUGOccupancyMsg::ConstPtr&
*/
void PlannerNode::OccupancyGridHandler(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    
    ROS_INFO("Updating map for planner");
    
    // LookupTransform function:  the transform between the global frame and the frame receiving occupancy grid message
    
    geometry_msgs::TransformStamped tfStamp = tfBuffer->lookupTransform(global_frame_id, msg->header.frame_id, ros::Time(0), ros::Duration(0.25));
    
    // Compute 2 corners to iterate through the vector in global
    // Assign msg->info.origin.position to p1 and init a p1t (the transformed origin got by doTransform())
    geometry_msgs::Point p1 = msg->info.origin.position, p1t;
    tf2::doTransform(p1, p1t, tfStamp);

    // updateMap(const mmpug_msgs::MMPUGOccupancyMsg& grid, geometry_msgs::Point& corner1);
    planner.updateMap(*msg, p1t);

    // new_occ_grid.data.clear();
    // for(const auto& p: msg->data)
    //     new_occ_grid.data.push_back((int8_t) p);

    // new_occ_grid.info = msg->info;
    // new_occ_grid.header = msg->header;

    // stats_pub.publish(new_occ_grid);

    initialized_map = true;
}

/**
 * PlannerNode plannerReqHandler
 * @param req: mmpug_msgs::RobotPlan::Request&
 * @param res: mmpug_msgs::RobotPlan::Response&
*/
void PlannerNode::plannerReqHandler( const geometry_msgs::PoseArray::ConstPtr& req ){ // former PoseStamped::ConstPtr& req

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
    float dt = d.toSec();
    ROS_INFO_STREAM("Planning routine complete, Time taken: " << dt);
    
    plan.header.frame_id = "global";
    plan.header.stamp = ros::Time::now();

    plan_publisher.publish(plan);
}

void PlannerNode::plannerCurrPoseHandler(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_odom = *msg;
}

// bool PlannerNode::plannerReqHandler(mmpug_msgs::RobotPlan::Request& req, mmpug_msgs::RobotPlan::Response& res){
//     geometry_msgs::PoseArray plan;
//     // naivePlanner based on request, computed a plan for the robot's path.
//     ros::Time s1 = ros::Time::now();
//     int planLength = planner.naivePlanner(req.robot_pose, req.goal_location, plan);
//     ros::Time s2 = ros::Time::now();
//     ros::Duration d = s2 - s1;
//     float dt = d.toSec();
//     ROS_INFO_STREAM("Planning routine complete, Time taken: " << dt);
//     plan.header.frame_id = "global";
//     plan.header.stamp = ros::Time::now();
//     // If successful, set response.
//     if(planLength > 0){
//         res.plan_successful = true;
//         res.plan_length = planLength;
//         // plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 4);
//         res.plan = plan;
//     } else {
//         res.plan_successful = false;
//         res.plan_length = 0;
//     }
//     return true;
// }

//  void PlannerNode::TargetHandler(const geometry_msgs::Pose::ConstPtr& msg){
//     if(!init_targets){
//         ROS_INFO("Updating targets for planner");
//         planner.setTargets(*msg);
//         init_targets = true;
//     }
// }
// void PlannerNode::OdometryHandler(const nav_msgs::Odometry::ConstPtr& msg){
//     if(!init_robot_pose){
//         // ROS_INFO("Updating robot pose for planner");
//         planner.setRobotLocation(msg->pose.pose);
//         // init_robot_pose = true;
//     }
// }

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