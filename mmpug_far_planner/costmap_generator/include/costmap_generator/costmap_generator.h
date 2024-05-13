#ifndef _COSTMAP_GEN_H_
#define _COSTMAP_GEN_H_

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/OccupancyGrid.h>
// #include <mmpug_msgs/MMPUGOccupancyMsg.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

class CostMapGenerator{
    private:
        float ros_rate;
        std::string global_frame_id, robot_name, robot_frame_id;
        bool debug, map_frame_global;
        
        ros::Subscriber odometry_sub, point_cloud_sub;
        ros::Publisher terrain_map_pub, occupancy_grid_pub;

        pcl::PointCloud<pcl::PointXYZI>::Ptr inputlaserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr outputCloud;

        std::string costmap_topic, costmap_update_topic, obstacles_topic, polygon_marker_topic;
        std::string odom_topic, point_cloud_topic;
        bool newlaserCloud, newOdometry;
        sensor_msgs::PointCloud2 debug_cloud1, outputCloudMsg;
        
        int occupied_min_value, inflationSize;

        float vehicleRoll, vehiclePitch, vehicleYaw, cosYaw, sinYaw;
        float vehicleX, vehicleY, vehicleZ;

        float rc_height, spot_height;
        double laserCloudTime;

        double quantileZ, lowerBoundZ, upperBoundZ, disRatioZ;
        int voxelPointUpdateThre, terrainVoxelShiftX, terrainVoxelShiftY, terrainVoxelHalfWidth;

        // terrain voxel parameters
        float terrainVoxelSize, scanVoxelSize;
        int terrainVoxelWidth, terrainVoxelNum;

        // planar voxel parameters
        float planarVoxelSize ;
        int planarVoxelWidth, planarVoxelNum;
        int planarVoxelHalfWidth;

        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStacked;
        pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud; 
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev; 
        pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElevImu;
        
        nav_msgs::OccupancyGrid occ_grid;
        // mmpug_msgs::MMPUGOccupancyMsg occ_grid;
        
        float min_range_x, max_range_x, min_range_y, max_range_y, min_range_z, max_range_z;

        //PCL Cloud Processing Objects
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        pcl::CropBox<pcl::PointXYZI> boxFilter, boxFilter2;
        float minX, maxX, minY, maxY, minZ, maxZ; 

        ros::Time odom_time, laser_time;

    public:
        CostMapGenerator();
        ~CostMapGenerator();
        
        ros::NodeHandle nh;
        ros::NodeHandle p_nh = ros::NodeHandle("~");

        void CloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg);
        void OdometryHandler(const nav_msgs::OdometryConstPtr& odom);
        void OdomVelodyneHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2, const nav_msgs::OdometryConstPtr& odom);
        void Run();

};

#endif
