/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  class to wrap c++ grasp planner classes in python
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#include <manipulation_optimizer/GraspPlanner.hpp>
#include <manipulation_optimizer/AgileGraspHandles.hpp>

using namespace manipulation_optimizer;

std_msgs::Int64 GraspPlanner::getAgileGraspCandidateSize(){
	return agile_grasp_handles_.size();
}

geometry_msgs::Vector3 GraspPlanner::getAgileGraspCandidateCenterAt(const std_msgs::Int64& idx){
	return agile_grasp_handles_.getCenter(idx);
}

geometry_msgs::Vector3 GraspPlanner::getAgileGraspCandidateAxisAt(const std_msgs::Int64& idx){
	return agile_grasp_handles_.getAxisVector(idx);
}

geometry_msgs::Vector3 GraspPlanner::getAgileGraspCandidateApproachAt(const std_msgs::Int64& idx){
	return agile_grasp_handles_.getApproachVector(idx);
}

void GraspPlanner::configureAgilePlannerLocalizer(const geometry_msgs::Pose& camera_pose, const std_msgs::Int64& num_samples, const std_msgs::Int64& num_threads,
				const std_msgs::Float64& taubin_radius, const std_msgs::Float64& hand_radius){
	agile_grasp_planner_.configureGraspLocalizer(camera_pose, num_samples, num_threads, taubin_radius, hand_radius);
}

void GraspPlanner::configureAgilePlannerWorkspace(const std_msgs::Float64MultiArray workspace){
	agile_grasp_planner_.setWorkspaceDimension(workspace);
}

void GraspPlanner::configureAgilePlannerHand(const std_msgs::Float64& finger_width, const std_msgs::Float64& hand_outer_diameter, const std_msgs::Float64& hand_depth,
		const std_msgs::Float64& init_bite, const std_msgs::Float64& hand_height){
	agile_grasp_planner_.configureHand(finger_width, hand_outer_diameter, hand_depth, init_bite, hand_height);
}

void GraspPlanner::computeAgileGraspFromPointCloud(const std_msgs::String& pointcloud_name, const std_msgs::String& svm_file_name, const std_msgs::Int64& min_inliers){
	agile_grasp_planner_.computeGraspFromPointCloud(pointcloud_name, svm_file_name, min_inliers);
	agile_grasp_handles_=agile_grasp_planner_.getCandidateGrasps();
}