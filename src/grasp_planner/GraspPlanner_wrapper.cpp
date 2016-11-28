/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  wrapper for the GraspPlanner class
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#include <boost/python.hpp>
#include <string>
#include <ros/serialization.h>

#include <manipulation_optimizer/GraspPlanner.hpp>
#include <manipulation_optimizer/AgileGraspHandles.hpp>

/* Read a ROS message from a serialized string.
  */
template <typename M>
M from_python(const std::string str_msg)
{
  size_t serial_size = str_msg.size();
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  for (size_t i = 0; i < serial_size; ++i)
  {
    buffer[i] = str_msg[i];
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  M msg;
  ros::serialization::Serializer<M>::read(stream, msg);
  return msg;
}

/* Write a ROS message into a serialized string.
*/
template <typename M>
std::string to_python(const M& msg)
{
  size_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, msg);
  std::string str_msg;
  str_msg.reserve(serial_size);
  for (size_t i = 0; i < serial_size; ++i)
  {
    str_msg.push_back(buffer[i]);
  }
  return str_msg;
}

class GraspPlannerWrapper : public manipulation_optimizer::GraspPlanner
{
  public:
    GraspPlannerWrapper() : GraspPlanner() {}

  std::string getAgileGraspCandidateSize(){
    std_msgs::Int64 s=GraspPlanner::getAgileGraspCandidateSize();

    return to_python(s);
  }

  std::string getAgileGraspCandidateCenterAt(const std::string& str_idx){
    std_msgs::Int64 idx=from_python<std_msgs::Int64>(str_idx);
    geometry_msgs::Vector3 s=GraspPlanner::getAgileGraspCandidateCenterAt(idx);

    return to_python(s);
  }

  std::string getAgileGraspCandidateAxisAt(const std::string& str_idx){
    std_msgs::Int64 idx=from_python<std_msgs::Int64>(str_idx);
    geometry_msgs::Vector3 s=GraspPlanner::getAgileGraspCandidateAxisAt(idx);

    return to_python(s);
  }

  std::string getAgileGraspCandidateApproachAt(const std::string& str_idx){
    std_msgs::Int64 idx=from_python<std_msgs::Int64>(str_idx);
    geometry_msgs::Vector3 s=GraspPlanner::getAgileGraspCandidateApproachAt(idx);

    return to_python(s);
  }

  void configureAgilePlannerLocalizer(const std::string& str_camera_pose, const std::string& str_num_samples, const std::string& str_num_threads,
                                  const std::string& str_taubin_radius, const std::string& str_hand_radius){

    geometry_msgs::Pose camera_pose=from_python<geometry_msgs::Pose>(str_camera_pose);
    std_msgs::Int64 num_samples=from_python<std_msgs::Int64>(str_num_samples);
    std_msgs::Int64 num_threads=from_python<std_msgs::Int64>(str_num_threads);
    std_msgs::Float64 taubin_radius=from_python<std_msgs::Float64>(str_taubin_radius);
    std_msgs::Float64 hand_radius=from_python<std_msgs::Float64>(str_hand_radius);

    GraspPlanner::configureAgilePlannerLocalizer(camera_pose, num_samples, num_threads, taubin_radius, hand_radius);
  }

  void configureAgilePlannerWorkspace(const std::string& str_workspace){
    std_msgs::Float64MultiArray workspace=from_python<std_msgs::Float64MultiArray>(str_workspace);
    GraspPlanner::configureAgilePlannerWorkspace(workspace);
  }

  void configureAgilePlannerHand(const std::string& str_finger_width, const std::string& str_hand_outer_diameter, const std::string& str_hand_depth,
                    const std::string& str_init_bite, const std::string& str_hand_height){

    std_msgs::Float64 finger_width=from_python<std_msgs::Float64>(str_finger_width);
    std_msgs::Float64 hand_outer_diameter=from_python<std_msgs::Float64>(str_hand_outer_diameter);
    std_msgs::Float64 hand_depth=from_python<std_msgs::Float64>(str_hand_depth);
    std_msgs::Float64 init_bite=from_python<std_msgs::Float64>(str_init_bite);
    std_msgs::Float64 hand_height=from_python<std_msgs::Float64>(str_hand_height);

    GraspPlanner::configureAgilePlannerHand(finger_width, hand_outer_diameter, hand_depth, init_bite, hand_height);
  }

  void computeAgileGraspFromPointCloud(const std::string& str_pointcloud_name, const std::string& str_svm_file_name, const std::string& str_min_inliers){
    std_msgs::String pointcloud_name=from_python<std_msgs::String>(str_pointcloud_name);
    std_msgs::String svm_file_name=from_python<std_msgs::String>(str_svm_file_name);
    std_msgs::Int64 min_inliers=from_python<std_msgs::Int64>(str_min_inliers);

    GraspPlanner::computeAgileGraspFromPointCloud(pointcloud_name, svm_file_name, min_inliers);
  }
};

BOOST_PYTHON_MODULE(_GraspPlanner_wrapper_cpp)
{
  boost::python::class_<GraspPlannerWrapper>("GraspPlannerWrapper", boost::python::init<>())
    .def("getAgileGraspCandidateSize", &GraspPlannerWrapper::getAgileGraspCandidateSize)
    .def("getAgileGraspCandidateCenterAt", &GraspPlannerWrapper::getAgileGraspCandidateCenterAt)
    .def("getAgileGraspCandidateAxisAt", &GraspPlannerWrapper::getAgileGraspCandidateAxisAt)
    .def("getAgileGraspCandidateApproachAt", &GraspPlannerWrapper::getAgileGraspCandidateApproachAt)
    .def("configureAgilePlannerLocalizer", &GraspPlannerWrapper::configureAgilePlannerLocalizer)
    .def("configureAgilePlannerWorkspace", &GraspPlannerWrapper::configureAgilePlannerWorkspace)
    .def("configureAgilePlannerHand", &GraspPlannerWrapper::configureAgilePlannerHand)
    .def("computeAgileGraspFromPointCloud", &GraspPlannerWrapper::computeAgileGraspFromPointCloud)
    ;
}