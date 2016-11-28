/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  used to stream the correct data after perception
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#include "ros/ros.h"
#include <ros/package.h>
#include <string>
#include "sensor_msgs/PointCloud2.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl_conversions/pcl_conversions.h"
#include "manipulation_optimizer/AskPointCloud.h"

bool sendPointCloud(manipulation_optimizer::AskPointCloud::Request  &req, manipulation_optimizer::AskPointCloud::Response &res){
    std::string path = ros::package::getPath("manipulation_optimizer");
    std::string filename=path+"/data/models/poinclouds/"+req.object_name+".pcd";
    sensor_msgs::PointCloud2 pc;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_ptr) == -1){
        std::string error="Couldn't read file"+req.object_name+".pcd \n";
        PCL_ERROR(error.c_str());
        return false;
    }

    pcl::toROSMsg(*cloud_ptr, pc);
    pc.header.frame_id="/world";
    pc.header.stamp = ros::Time(0);
    res.point_cloud=pc;

    return true;

}

int main(int argc, char **argv){
 	ros::init(argc, argv, "ask_point_cloud_server");
 	ros::NodeHandle n;

 	ros::ServiceServer service=n.advertiseService("ask_point_cloud", sendPointCloud);
    ROS_INFO("Waiting for object name.");

    ros::spin();

 	return 0;
}