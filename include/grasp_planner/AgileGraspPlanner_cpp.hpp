/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  class containing functions from the agile_grasp grasp planner
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#ifndef MANIPULATION_OPTIMIZER_AGILE_GRASP_PLANNER_CPP_HPP
#define MANIPULATION_OPTIMIZER_AGILE_GRASP_PLANNER_CPP_HPP

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>

#include <agile_grasp/Grasp.h>
#include <agile_grasp/localization.h>

#include "AgileGraspHandles.hpp"

namespace manipulation_optimizer {

class AgileGraspPlanner_cpp
{
  public:
    void configureGraspLocalizer(const geometry_msgs::Pose& camera_pose, const std_msgs::Int64& num_samples, const std_msgs::Int64& num_threads,
    							const std_msgs::Float64& taubin_radius, const std_msgs::Float64& hand_radius);

	void setWorkspaceDimension(const std_msgs::Float64MultiArray workspace);

	void configureHand(const std_msgs::Float64& finger_width, const std_msgs::Float64& hand_outer_diameter, const std_msgs::Float64& hand_depth,
					const std_msgs::Float64& init_bite, const std_msgs::Float64& hand_height);

	void computeGraspFromPointCloud(const std_msgs::String& pointcloud_name, const std_msgs::String& svm_file_name, const std_msgs::Int64& min_inliers);

	//void computeGraspFromStreamData();

	AgileGraspHandles getCandidateGrasps();

  private:
  	Localization loc_;
	AgileGraspHandles handles_;
};

} // namespace manipulation_optimizer

#endif //MANIPULATION_OPTIMIZER_AGILE_GRASP_PLANNER_CPP_HPP