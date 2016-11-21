/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  class containing functions from the agile_grasp grasp planner
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#include <manipulation_optimizer/AgileGraspPlanner_cpp.hpp>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

using namespace manipulation_optimizer;

void AgileGraspPlanner_cpp::configureGraspLocalizer(const geometry_msgs::Pose& camera_pose, const std_msgs::Int64& num_samples, const std_msgs::Int64& num_threads,
							const std_msgs::Float64& taubin_radius, const std_msgs::Float64& hand_radius){

	Eigen::Matrix4d camera_tf;
	Eigen::Affine3d affine_tf;

	tf::poseMsgToEigen(camera_pose, affine_tf);
	camera_tf=affine_tf.matrix();

	loc_=Localization(num_samples.data, true, false);

	loc_.setNumSamples(num_samples.data);
	loc_.setCameraTransforms(camera_tf, camera_tf);
	loc_.setNeighborhoodRadiusTaubin(taubin_radius.data);
    loc_.setNeighborhoodRadiusHands(hand_radius.data);
    Eigen::VectorXd workspace(6);
    workspace << -10, 10, -10, 10, -10, 10;
    loc_.setWorkspace(workspace);

}

void AgileGraspPlanner_cpp::setWorkspaceDimension(const std_msgs::Float64MultiArray workspace){
	Eigen::VectorXd w_vec(6);
	w_vec<<workspace.data[0], workspace.data[1], workspace.data[2], workspace.data[3], workspace.data[4], workspace.data[5], workspace.data[6];
	loc_.setWorkspace(w_vec);

}

void AgileGraspPlanner_cpp::configureHand(const std_msgs::Float64& finger_width, const std_msgs::Float64& hand_outer_diameter, const std_msgs::Float64& hand_depth,
									const std_msgs::Float64& init_bite, const std_msgs::Float64& hand_height){
	loc_.setFingerWidth(finger_width.data);
    loc_.setHandOuterDiameter(hand_outer_diameter.data);
    loc_.setHandDepth(hand_depth.data);
    loc_.setInitBite(init_bite.data);
    loc_.setHandHeight(hand_height.data);
}

void AgileGraspPlanner_cpp::computeGraspFromPointCloud(const std_msgs::String& pointcloud_name, const std_msgs::String& svm_file_name, const std_msgs::Int64& min_inliers){
	std::vector<GraspHypothesis> hands=loc_.localizeHands(pointcloud_name.data, "");
    std::vector<GraspHypothesis> antipodal_hands=loc_.predictAntipodalHands(hands, svm_file_name.data);
	handles_=AgileGraspHandles(loc_.findHandles(antipodal_hands, min_inliers.data, 0.005));
}

AgileGraspHandles AgileGraspPlanner_cpp::getCandidateGrasps(){
	return handles_;
}
