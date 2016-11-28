/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  class to wrap c++ grasp planner classes in python
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#ifndef MANIPULATION_OPTIMIZER_GRASP_PLANNER_HPP
#define MANIPULATION_OPTIMIZER_GRASP_PLANNER_HPP

//#include "AgileGraspHandles.hpp"
#include "AgileGraspPlanner_cpp.hpp"

namespace manipulation_optimizer {

class GraspPlanner
{
  public:
    std_msgs::Int64 getAgileGraspCandidateSize();
    geometry_msgs::Vector3 getAgileGraspCandidateCenterAt(const std_msgs::Int64& idx);
    geometry_msgs::Vector3 getAgileGraspCandidateAxisAt(const std_msgs::Int64& idx);
    geometry_msgs::Vector3 getAgileGraspCandidateApproachAt(const std_msgs::Int64& idx);

    void configureAgilePlannerLocalizer(const geometry_msgs::Pose& camera_pose, const std_msgs::Int64& num_samples, const std_msgs::Int64& num_threads,
    				const std_msgs::Float64& taubin_radius, const std_msgs::Float64& hand_radius);
    void configureAgilePlannerWorkspace(const std_msgs::Float64MultiArray workspace);
    void configureAgilePlannerHand(const std_msgs::Float64& finger_width, const std_msgs::Float64& hand_outer_diameter, const std_msgs::Float64& hand_depth,
			const std_msgs::Float64& init_bite, const std_msgs::Float64& hand_height);
    void computeAgileGraspFromPointCloud(const std_msgs::String& pointcloud_name, const std_msgs::String& svm_file_name, const std_msgs::Int64& min_inliers);

  private:
    AgileGraspHandles agile_grasp_handles_;
    AgileGraspPlanner_cpp agile_grasp_planner_;
};

} // namespace manipulation_optimizer

#endif // MANIPULATION_OPTIMIZER_GRASP_PLANNER_HPP

