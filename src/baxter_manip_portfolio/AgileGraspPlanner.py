#!/usr/bin/env python
"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains an implementation of the abstract method type 'GraspPlanner' as defined in deliverable D4.1.
    This implementation utilizes the grasp planner provided by agile_grasp.

    @author: Silvia Cruciani (cruciani@kth.se)
"""
from manipDreamBed.MethodTypes import GraspPlanner

from manipulation_optimizer import GraspPlannerWrap
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose


class AgileGraspPlanner(GraspPlanner):
    """ A wrapper class for agile_grasp that fulfills the GraspPlanner interface. """
    def __init__(self):
        self._graspPlanner = GraspPlannerWrap()
        self._svm_file_name = ""
        self._min_inliers = 3

    def getName(self):
        return "AgileGraspPlanner"

    def getParameters(self):
        # TODO
        return None

    def getConditionals(self):
        # TODO
        return None

    def getForbiddenConfigurations(self):
        # TODO
        return None

    def initialize(self):
        # TODO
        self.initializeDefaultParam()
        pass

    def allocateResources(self):
        pass

    def releaseResources(self):
        pass

    def plan(self, object):
        """Compute a grasp candidate.
        """
        pcd_name = "../../data/models/pointclouds/" + object.name + ".pcd"
        self._graspPlanner.computeAgileGraspFromPointCloud(String(pcd_name), String(self._svm_name), Int64(self._min_inliers))

        axis_vec = self._graspPlanner.getAgileGraspCandidateAxisAt(Int64(0))
        approach_vec = self._getAgileGraspCandidateApproachAt(Int64(0))
        #other???? this is what I can provide: the center of the grasp (grasp position between the end of the finger tips),
        #the width of the object contained in the grasp, the center of the grasp projected onto the back of the hand.

        return (approach_vec, axis_vec)

    def initializeDefaultParam(self):
        """Initialise the grasp planner with default values
        """
        camera_pose = Pose()
        camera_pose.orientation.w = 1
        num_samples = Int64(400)
        num_threads = Int64(1)
        tabin_radius = Float64(0.03)
        hand_radius = Float64(0.08)

        self._graspPlanner.configureAgilePlannerLocalizer(camera_pose, num_samples, num_threads, tabin_radius, hand_radius)

        finger_width = Float64(0.01)
        hand_outer_diameter = Float64(0.09)
        hand_depth = Float64(0.06)
        init_bite = Float64(0.01)
        hand_height = Float64(0.02)

        self._graspPlanner.configureAgilePlannerHand(finger_width, hand_outer_diameter, hand_depth, init_bite, hand_height)

        pass
