/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  class containing possible grasp candidates obtained from the agile_grasp pkg
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#include <manipulation_optimizer/AgileGraspHandles.hpp>

using namespace manipulation_optimizer;

AgileGraspHandles::AgileGraspHandles(){}

AgileGraspHandles::~AgileGraspHandles(){}

AgileGraspHandles::AgileGraspHandles(std::vector<Handle> handles){
	handles_=handles;
}

std_msgs::Int64 AgileGraspHandles::size(){
  std_msgs::Int64 s;
  s.data= handles_.size();
  return s;
}

geometry_msgs::Vector3 AgileGraspHandles::getCenter(const std_msgs::Int64& idx){
	geometry_msgs::Vector3 center;
	Eigen::Vector3d vec=handles_[idx.data].getCenter();
	center.x=vec(0);
	center.y=vec(1);
	center.z=vec(2);
	return center;
}

geometry_msgs::Vector3 AgileGraspHandles::getAxisVector(const std_msgs::Int64& idx){
	geometry_msgs::Vector3 center;
	Eigen::Vector3d vec=handles_[idx.data].getAxis();
	center.x=vec(0);
	center.y=vec(1);
	center.z=vec(2);
	return center;
}

geometry_msgs::Vector3 AgileGraspHandles::getApproachVector(const std_msgs::Int64& idx){
	geometry_msgs::Vector3 center;
	Eigen::Vector3d vec=handles_[idx.data].getApproach();
	center.x=vec(0);
	center.y=vec(1);
	center.z=vec(2);
	return center;
}