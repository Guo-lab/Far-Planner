#include "costmap_generator/costmap_generator.h"

CostMapGenerator::CostMapGenerator():nh(){

    p_nh.getParam("ros_rate", ros_rate);
    p_nh.getParam("debug", debug);
    p_nh.getParam("global_frame_id", global_frame_id);

    p_nh.getParam("rc_height", rc_height);
    p_nh.getParam("spot_height", spot_height);

    p_nh.getParam("scanVoxelSize", scanVoxelSize);
    p_nh.getParam("terrainVoxelWidth", terrainVoxelWidth);
    p_nh.getParam("quantileZ", quantileZ);
    p_nh.getParam("inflationSize", inflationSize);

    p_nh.getParam("lowerBoundZ", lowerBoundZ);
    p_nh.getParam("upperBoundZ", upperBoundZ);
    p_nh.getParam("disRatioZ", disRatioZ);

    p_nh.getParam("min_range_y", min_range_y);
    p_nh.getParam("max_range_y", max_range_y);
    
    p_nh.getParam("min_range_x", min_range_x);
    p_nh.getParam("max_range_x", max_range_x);
    
    min_range_z = lowerBoundZ;
    max_range_z = upperBoundZ;

    p_nh.getParam("robot_name", robot_name);
    p_nh.getParam("is_map_frame_global", map_frame_global);

    robot_frame_id = robot_name + "_sensor_init";
    
    p_nh.getParam("costmap_topic", costmap_topic);
    p_nh.getParam("costmap_update_topic", costmap_update_topic);
    p_nh.getParam("obstacles_topic", obstacles_topic);
    p_nh.getParam("polygon_marker_topic", polygon_marker_topic);


    p_nh.getParam("odom_topic", odom_topic);
    p_nh.getParam("point_cloud_topic", point_cloud_topic);
    p_nh.getParam("occupied_min_value", occupied_min_value);

    newlaserCloud = false;
    newOdometry = false;

    // planar voxel parameters
    planarVoxelSize = scanVoxelSize;
    planarVoxelWidth = terrainVoxelWidth;
    planarVoxelHalfWidth = (planarVoxelWidth ) / 2;
    planarVoxelNum = planarVoxelWidth * planarVoxelWidth;

    inputlaserCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    laserCloudStacked = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    outputCloud =  pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloudElev = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    terrainCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

}

CostMapGenerator::~CostMapGenerator(){}

void CostMapGenerator::Run(){

    ROS_INFO("Terrain Map Node started,  Debug: %d", debug);

    // Create ROS Subscribers for Velodyne, Odometry and Images
    // point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/rc1/velodyne_cloud_registered", 2, &CostMapGenerator::CloudHandler, this);
    point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(point_cloud_topic, 1, &CostMapGenerator::CloudHandler, this);
    odometry_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, &CostMapGenerator::OdometryHandler, this);

    // Creater Publisher for annotated terrain mpa
    terrain_map_pub = nh.advertise<sensor_msgs::PointCloud2>("terrain/pointcloud",2);

    // occupancy_grid_pub = nh.advertise<mmpug_msgs::MMPUGOccupancyMsg>(costmap_topic, 1, true);
    occupancy_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(costmap_topic, 1, true);
    
    occ_grid.info.resolution = scanVoxelSize;
    occ_grid.info.height = planarVoxelWidth;
    occ_grid.info.width = planarVoxelWidth;
    

    // std::vector<int8_t> data(planarVoxelWidth * planarVoxelWidth);
    std::vector<int32_t> data(planarVoxelWidth * planarVoxelWidth);
    std::fill(data.begin(), data.end(), occupied_min_value/2);
    
    for (auto &cell : data)
    {
        occ_grid.data.push_back(static_cast<signed char>(cell));
    }
    // occ_grid.data = data;

    ros::spinOnce(); 
    ros::Rate rate(ros_rate);

    int count = 0;
    float planarVoxelElev[planarVoxelNum] = {0};
    std::vector<float> planarPointElev[planarVoxelNum];


    while(ros::ok()){
        if(newlaserCloud){
            if(newOdometry){
                
                std::fill(occ_grid.data.begin(), occ_grid.data.end(), occupied_min_value/2);
                
                for (int i = 0; i < planarVoxelNum; i++) {
                    planarPointElev[i].clear();
                }
                pcl::PointXYZI point0, point;

                int terrainCloudSize = terrainCloud->points.size();
                for (int i = 0; i < terrainCloudSize; i++) {

                    point = terrainCloud->points[i];

                    // point.x = point0.z;
                    // point.y = point0.x;
                    // point.z = point0.y;

                    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));

                    if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {

                        int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
                        int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

                        if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
                        if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

                        for (int dX = -1; dX <= 1; dX++) {
                            for (int dY = -1; dY <= 1; dY++) {
                                if (indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 && indY + dY < planarVoxelWidth) {
                                    planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
                                }
                            }
                        }
                    }
                }

                for (int i = 0; i < planarVoxelNum; i++) {

                    int planarPointElevSize = planarPointElev[i].size();

                    if (planarPointElevSize > 0) {
                        float minZ = 1000.0;
                        int minID = -1;
                        for (int j = 0; j < planarPointElevSize; j++) {
                            if (planarPointElev[i][j] < minZ) {
                                minZ = planarPointElev[i][j];
                                minID = j;
                            }
                        }

                        if (minID != -1) {
                            planarVoxelElev[i] = planarPointElev[i][minID];
                        }
                    }
                }

                terrainCloudElev->clear();
                int terrainCloudElevSize = 0;

                for (int i = 0; i < terrainCloudSize; i++) {
                    point = terrainCloud->points[i];

                    // point.x = point0.z;
                    // point.y = point0.x;
                    // point.z = point0.y;

                    float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX) + (point.y - vehicleY) * (point.y - vehicleY));
                    if (point.z - vehicleZ > lowerBoundZ - disRatioZ * dis && point.z - vehicleZ < upperBoundZ + disRatioZ * dis) {

                        int indX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;
                        int indY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth;

                        if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
                        if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

                        if (indX >= 0 && indX <= planarVoxelWidth && indY >= 0 && indY <=planarVoxelWidth) {

                            float disZ = fabs(point.z - planarVoxelElev[planarVoxelWidth * indX + indY]);
                            // if (disZ <= rc_height) {

                                terrainCloudElev->push_back(point);

                                terrainCloudElev->points[terrainCloudElevSize].x = point.x;
                                terrainCloudElev->points[terrainCloudElevSize].y = point.y;
                                terrainCloudElev->points[terrainCloudElevSize].z = point.z;
                                terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;

                                terrainCloudElevSize++;
                            // }

                            int occX = int((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth - 1;
                            int occY = int((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) + planarVoxelHalfWidth - 1;
                            if(occX >= 0 && occX < planarVoxelWidth && occY >= 0 && occY < planarVoxelWidth)
                            {
                                if(disZ > rc_height){
                                    occ_grid.data.at(occX + (occY * planarVoxelWidth)) = occupied_min_value;
                                    
                                    for (int dx = -inflationSize; dx <= inflationSize; ++dx) {
                                        for (int dy = -inflationSize; dy <= inflationSize; ++dy) {
                                            if ((occX + dx)>= 0 && (occX + dx) < planarVoxelWidth && (occY + dy) >= 0 && (occY + dy) < planarVoxelWidth) {
                                                occ_grid.data.at((occY + dy) * planarVoxelWidth + (occX + dx))= occupied_min_value;
                                            }
                                        }
                                    }
                                }
                                else{
                                    if(occ_grid.data.at(occX + (occY * planarVoxelWidth)) != occupied_min_value)
                                        occ_grid.data.at(occX + (occY * planarVoxelWidth)) = 0;
                                }
                            }
                            else{
                                std::cout << "Here:? " << occX << occY<< std::endl;
                            }
                            
                        }
                    }
                }

                if(map_frame_global)
                    occ_grid.header.frame_id = global_frame_id;
                else
                    occ_grid.header.frame_id = robot_frame_id;

                occ_grid.header.stamp = ros::Time().fromSec(laserCloudTime);
                geometry_msgs::Pose p;
                p.position.x = vehicleX - planarVoxelHalfWidth * planarVoxelSize;
                p.position.y = vehicleY - planarVoxelHalfWidth * planarVoxelSize;
                p.position.z = 0;
                occ_grid.info.origin = p;

                if(count % 5 == 0){
                    occupancy_grid_pub.publish(occ_grid);
                    count = 0;
                }
                
                count++;
                pcl::toROSMsg(*terrainCloudElev, outputCloudMsg);
                outputCloudMsg.header.stamp = ros::Time().fromSec(laserCloudTime);
                // outputCloudMsg.header.stamp = ros::Time::now();
                if(map_frame_global)
                    outputCloudMsg.header.frame_id = global_frame_id;
                else
                    outputCloudMsg.header.frame_id = robot_frame_id;
                terrain_map_pub.publish(outputCloudMsg);

                // newlaserCloud = false;
                // newOdometry = false;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
}

void CostMapGenerator::CloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg){
        
    inputlaserCloud->clear();
    pcl::fromROSMsg(*msg, *inputlaserCloud);
    laser_time = msg->header.stamp;

        //TODO: Remove this when we switch to biodoometry
    laserCloud->clear();
    for(const auto& p: inputlaserCloud->points){
        pcl::PointXYZI p1;
        
        p1.x = p.z;
        p1.y = p.x;
        p1.z = p.y;

        // p1.x = p.x;
        // p1.y = p.y;
        // p1.z = p.z;

        float dist = sqrt((p1.x - vehicleX)*(p1.x - vehicleX) + (p1.y - vehicleY)*(p1.y - vehicleY) + (p1.z - vehicleZ)*(p1.z - vehicleZ));
        if(dist > 0.35){
            laserCloud->points.push_back(p1);
        }
        // else{
        //     std::cout << "I GOT HERE: " << std::endl;
        // }
    }

    *terrainCloud = (*terrainCloud) + (*laserCloud);

    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, -1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(terrainCloud);
    boxFilter.filter(*laserCloudStacked);

    sor.setInputCloud(laserCloudStacked);
    sor.setLeafSize(planarVoxelSize, planarVoxelSize, planarVoxelSize);

    terrainCloud->clear();
    sor.filter(*terrainCloud);
    laserCloudTime = msg->header.stamp.toSec();
    newlaserCloud = true;
    
}

void CostMapGenerator::OdometryHandler(const nav_msgs::OdometryConstPtr& odom){

    double roll, pitch, yaw;
    geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

    vehicleRoll = roll;
    vehiclePitch = pitch;
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;

    maxX =  max_range_x + vehicleX; 
    minX =  min_range_x + vehicleX;
    maxY =  max_range_y + vehicleY;
    minY =  min_range_y + vehicleY;
    maxZ =  max_range_z + vehicleZ; 
    minZ =  min_range_z + vehicleZ;

    cosYaw = cos(vehicleYaw);
    sinYaw = sin(vehicleYaw);

    odom_time = odom->header.stamp;
    newOdometry = true;

}

int main(int argc, char** argv) {

    ROS_INFO("Starting Costmap Generator ROS Node for TEB planner");
    ros::init(argc, argv, "TEB planner Costmap Gen Node");
    CostMapGenerator node;
    node.Run();
    return 0;
}