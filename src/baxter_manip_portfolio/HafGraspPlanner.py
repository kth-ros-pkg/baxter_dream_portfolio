#!/usr/bin/env python
"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains an implementation of the abstract method type 'GraspPlanner' as defined in deliverable D4.1.
    This implementation utilizes the grasp planner provided by haf_grasping.

    @author: Silvia Cruciani (cruciani@kth.se)
"""
from manipulation_dreambed.MethodTypes import (GraspPlanner, GraspResult)
import rospy
import sys
import tf
import actionlib
import numpy
from rospy import Duration
from geometry_msgs.msg import (Point, Vector3)
from std_msgs.msg import String
from haf_grasping.msg import (CalcGraspPointsServerGoal, CalcGraspPointsServerAction)
from sensor_msgs.msg import PointCloud2
from baxter_dream_portfolio.srv import AskPointCloud


class HafGraspPlanner(GraspPlanner):
    """ A class for haf_grasping that fulfills the GraspPlanner interface. """
    def __init__(self, parameters):
        self._graspsearchcenter = Point() #center for searching for grasps
        self._approach_vector = Vector3() #defines the direction from where a grasp should be executed
        self._grasp_search_size_x = 0 #the size (x direction) where grasps are really calculated (in each direction 7cm more are needed for feature calculation!
        self._grasp_search_size_y = 0 #the size (y direction) where grasps are really calculated (in each direction 7cm more are needed for feature calculation!
        #self._max_grasp_search_size_x = 18 #x-limit for grasp search area size
        #self._max_grasp_search_size_y = 30 #y-limit for grasp search area size
        self._grasp_calculation_time_max = Duration()   #max time used for grasp calculation (sec) before result is returned
        self._show_only_best_grasp = False
        #self._base_frame_default = "base_link"
        self._gripper_opening_width = 0 #defines pre-grasp gripper opening width
        rospy.wait_for_service('ask_point_cloud')
        pass

    def getName(self):
        return "HafGraspPlanner"

    def getParameters(self):
        # TODO
        return {}

    def getConditionals(self):
        # TODO
        return None

    def getForbiddenConfigurations(self):
        # TODO
        return None

    def initialize(self):
        self.initializeDefaultParam()
        pass

    def allocateResources(self):
        pass

    def releaseResources(self):
        pass

    def plan(self, object, context, paramPrefix, parameters):

        try:
            ask_point_cloud = rospy.ServiceProxy('ask_point_cloud', AskPointCloud)
            resp = ask_point_cloud(object.name)

            eulerAngles = tf.transformations.euler_from_quaternion(object.pose.orientation)
            objectToWorld = tf.transformations.compose_matrix(translate=object.pose.position, angles=eulerAngles)

            # compute grasp through actionlib
            ac = actionlib.SimpleActionClient('calc_grasppoints_svm_action_server', CalcGraspPointsServerAction)

            ac.wait_for_server()

            #goal definition
            goal = CalcGraspPointsServerGoal()

            goal.graspinput.input_pc = resp.point_cloud
            goal.graspinput.grasp_area_center = self._graspsearchcenter
            #set grasp approach vector
            approach_num = numpy.dot(objectToWorld[:3, :3].transpose(), numpy.array([self._approach_vector.x, self._approach_vector.y, self._approach_vector.z]))
            goal.graspinput.approach_vector = Vector3(approach_num[0], approach_num[1], approach_num[2])
            #set size of grasp search area
            goal.graspinput.grasp_area_length_x = self._grasp_search_size_x + 14
            goal.graspinput.grasp_area_length_y = self._grasp_search_size_y + 14
            #set max grasp calculation time
            goal.graspinput.max_calculation_time = self._grasp_calculation_time_max
            #set if only best grasp should be visualized
            goal.graspinput.show_only_best_grasp = self._show_only_best_grasp
            #set pre-grasp gripper opening width (factor for scaling pc)
            goal.graspinput.gripper_opening_width = self._gripper_opening_width
            goal.graspinput.goal_frame_id = "/world"

            #send goal
            ac.send_goal(goal)

            finished_before_timeout = ac.wait_for_result(rospy.Duration.from_sec(50.0))
            if finished_before_timeout:
                print ("Haf grasp planner found a solution!")
                result = ac.get_result()
                grasp_point = result.graspOutput.averagedGraspPoint
                approach_vec = result.graspOutput.approachVector
                roll = result.graspOutput.roll

                grasp_point_v = numpy.array([grasp_point.x, grasp_point.y, grasp_point.z, 1])
                graspInWorldFrame = numpy.dot(objectToWorld, grasp_point_v)[:3]

                approach_vec_v = numpy.array([approach_vec.x, approach_vec.y, approach_vec.z])
                approachInWorldFrame = numpy.dot(objectToWorld[:3, :3], approach_vec_v)


                return GraspResult(approach_vector=approachInWorldFrame, grasp_position=graspInWorldFrame, orientation=roll)

            else:
                print ("Haf Grasp Planner action did not finish before the time out.")
                return None


        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def destroy(self):
        pass

    def initializeDefaultParam(self):
        """Initialise the grasp planner with default values
        """
        self._graspsearchcenter = Point()
        self._approach_vector = Vector3()
        self._approach_vector.z = 1
        self._grasp_search_size_x = 18
        self._grasp_search_size_y = 30
        self._grasp_calculation_time_max = Duration(40)
        self._show_only_best_grasp = False
        self._gripper_opening_width = 1
        self._point_cloud = PointCloud2()

if __name__ == "__main__":

    '''Initialize ros node'''
    rospy.init_node('HafGraspPlanner', anonymous=True)

