/**
 * @Author Silvia Cruciani (cruciani@kth.se)
 * @brief  class containing possible grasp candidates obtained from the agile_grasp pkg
 *
 * This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
 */

#ifndef MANIPULATION_OPTIMIZER_AGILE_GRASP_HANDLES_HPP
#define MANIPULATION_OPTIMIZER_AGILE_GRASP_HANDLES_HPP

#include <std_msgs/Int64.h>
#include <geometry_msgs/Vector3.h>

#include <agile_grasp/handle.h>
#include <vector>
#include <Eigen/Core>

namespace manipulation_optimizer {

class AgileGraspHandles
{
  public:
  	AgileGraspHandles();
  	~AgileGraspHandles();
  	AgileGraspHandles(std::vector<Handle> handles);
    std_msgs::Int64 size();
    geometry_msgs::Vector3 getCenter(const std_msgs::Int64& idx);
    geometry_msgs::Vector3 getAxisVector(const std_msgs::Int64& idx);
    geometry_msgs::Vector3 getApproachVector(const std_msgs::Int64& idx);
  private:
  	std::vector<Handle> handles_;
};

} // namespace manipulation_optimizer

#endif // MANIPULATION_OPTIMIZER_AGILE_GRASP_HANDLES_HPP
