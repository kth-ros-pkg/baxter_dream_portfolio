"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It wraps the class defined in GraspPlanner.hpp

    @author: Silvia Cruciani (cruciani@kth.se)
"""

from StringIO import StringIO

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose

from manipulation_optimizer._GraspPlanner_wrapper_cpp import GraspPlannerWrapper


class GraspPlannerWrap(object):
    def __init__(self):
        self._GraspPlannerWrap = GraspPlannerWrapper()

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

    def getAgileGraspCandidateSize(self):
        """Number of candidate grasps

        Return a std_msgs/Int64 instance.
        """
        str_size = self._GraspPlannerWrap.getAgileGraspCandidateSize()
        return self._from_cpp(str_size, Int64)

    def getAgileGraspCandidateCenterAt(self, idx):
        """Center of idx-th candidate grasp

        Return a geometry_msgs/Vector3 instance.

        Parameters
        ----------
        - idx: a std_msgs/Int64 instance.
        """
        if not isinstance(idx, Int64):
            rospy.ROSException('Argument is not a std_msgs/Int64')
        str_idx=self._to_cpp(idx)
        str_vec=self._GraspPlannerWrap.getAgileGraspCandidateCenterAt(str_idx)
        return self._from_cpp(str_vec, Vector3)

    def getAgileGraspCandidateApproachAt(self, idx):
        """Approach vector of idx-th candidate grasp

        Return a geometry_msgs/Vector3 instance.

        Parameters
        ----------
        - idx: a std_msgs/Int64 instance.
        """
        if not isinstance(idx, Int64):
            rospy.ROSException('Argument is not a std_msgs/Int64')
        str_idx=self._to_cpp(idx)
        str_vec=self._GraspPlannerWrap.getAgileGraspCandidateApproachAt(str_idx)
        return self._from_cpp(str_vec, Vector3)

    def getAgileGraspCandidateAxisAt(self, idx):
        """Axis vector of idx-th candidate grasp

        Return a geometry_msgs/Vector3 instance.

        Parameters
        ----------
        - idx: a std_msgs/Int64 instance.
        """
        if not isinstance(idx, Int64):
            rospy.ROSException('Argument is not a std_msgs/Int64')
        str_idx=self._to_cpp(idx)
        str_vec=self._GraspPlannerWrap.getAgileGraspCandidateAxisAt(str_idx)
        return self._from_cpp(str_vec, Vector3)

    def configureAgilePlannerLocalizer(self, camera_pose, num_samples, num_threads,
                                taubin_radius, hand_radius):
        """Configuration of the grasp localizer.

	Parameters
        ----------
        - camera_pose: geometry_msgs/Pose
        - num_samples: std_msgs/Int64 
        - num_threads: std_msgs/Int64
        - taubin_radius: std_msgs/Float64
        - hand_radius: std_msgs/Float64.
        """
	if not isinstance(camera_pose, Pose):
            rospy.ROSException('Argument 1 is not a geometry_msgs/Pose')
	if not isinstance(num_samples, Int64):
            rospy.ROSException('Argument 2 is not a std_msgs/Int64')
	if not isinstance(num_threads, Int64):
            rospy.ROSException('Argument 3 is not a std_msgs/Int64')
	if not isinstance(taubin_radius, Float64):
            rospy.ROSException('Argument 4 is not a std_msgs/Float64')
	if not isinstance(hand_radius, Float64):
            rospy.ROSException('Argument 5 is not a std_msgs/Float64')
	str_camera_pose=self._to_cpp(camera_pose)
	str_num_samples=self._to_cpp(num_samples)
	str_num_threads=self._to_cpp(num_threads)
	str_taubin_radius=self._to_cpp(taubin_radius)
	str_hand_radius=self._to_cpp(hand_radius)

	self._GraspPlannerWrap.configureAgilePlannerLocalizer(str_camera_pose, str_num_samples, str_num_threads, str_taubin_radius, str_hand_radius)

    def configureAgilePlannerWorkspace(self, workspace):
        """Set the dimension of the workspace of the Grasp Localizer.

	Parameters
        ----------
        - workspace: std_msgs/Float64MultiArray[6]
        """
        if not isinstance(workspace, Float64MultiArray):
	    rospy.logerr('Argument is not a std_msgs/Float64MultiArray')
	if not len(workspace.data)==6:
	    rospy.logerr('Argumet is not valid. Dimension must be 6.')	

	str_workspace=self._to_cpp(workspace)
	self._GraspPlannerWrap.configureAgilePlannerWorkspace(str_workspace)

    def configureAgilePlannerHand(self, finger_width, hand_outer_diameter, hand_depth, init_bite, hand_height):
        """Configure the hand used by the Grasp Localizer.

	Parameters
        ----------
        - finger_width: std_msgs/Float64
	- hand_outer_diameter: std_msgs/Float64
	- hand_depth: std_msgs/Float64
	- init_bite: std_msgs/Float64
	- hand_height: std_msgs/Float64
        """
	if not isinstance(finger_width, Float64):
	    rospy.ROSException('Argument 1 is not a std_msgs/Float64')
	if not isinstance(hand_outer_diameter, Float64):
	    rospy.ROSException('Argument 2 is not a std_msgs/Float64')
	if not isinstance(hand_depth, Float64):
	    rospy.ROSException('Argument 3 is not a std_msgs/Float64')
	if not isinstance(init_bite, Float64):
	    rospy.ROSException('Argument 4 is not a std_msgs/Float64')
	if not isinstance(hand_height, Float64):
	    rospy.ROSException('Argument 5 is not a std_msgs/Float64')

	str_finger_width=self._to_cpp(finger_width)
	str_hand_outer_diameter=self._to_cpp(hand_outer_diameter)
	str_hand_depth=self._to_cpp(hand_depth)
	str_init_bite=self._to_cpp(init_bite)
	str_hand_height=self._to_cpp(hand_height)

	self._GraspPlannerWrap.configureAgilePlannerHand(str_finger_width, str_hand_outer_diameter, str_hand_depth, str_init_bite, str_hand_height)

    def computeAgileGraspFromPointCloud(self, pointcloud_name, svm_file_name, min_inliers):
        """Compute a set of candidate grasps.

	Parameters
        ----------
        - pointcloud_name: std_msgs/String name of a .pcd file
	- svm_file_name: std_msgs/String name of the trained SVM to use
	- min_inliers: std_msgs/Int64
        """
	if not isinstance(pointcloud_name, String):
	    rospy.ROSException('Argument 1 is not a std_msgs/String')
	if not isinstance(svm_file_name, String):
	    rospy.ROSException('Argument 2 is not a std_msgs/String')
	if not isinstance(min_inliers, Int64):
	    rospy.ROSException('Argument 3 is not a std_msgs/Int64')
	
	str_pointcloud_name=self._to_cpp(pointcloud_name)
	str_svm_file_name=self._to_cpp(svm_file_name)
	str_min_inliers=self._to_cpp(min_inliers)

	self._GraspPlannerWrap.computeAgileGraspFromPointCloud(str_pointcloud_name, str_svm_file_name, str_min_inliers)

