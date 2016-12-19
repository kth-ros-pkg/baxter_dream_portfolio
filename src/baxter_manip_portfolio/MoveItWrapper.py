"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains an implementation of the method types 'ArmPlanner' and 'ArmController' as defined in deliverable D4.1.
    This implementation utilizes the motion planners provided by MoveIt!.

    @author: Joshua Haustein (haustein@kth.se)
"""

import yaml
import moveit_commander
import sys
import tf.transformations
import numpy
import MoveItUtils
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
from moveit_msgs.msg import (RobotTrajectory, RobotState)
from genpy import rostime
from manipulation_dreambed.MethodTypes import (PortfolioMethod, ArmPlanner, ArmController, Trajectory, Waypoint)
import manipulation_dreambed.ROSUtils as ROSUtils
from manipulation_dreambed.Context import (Pose as ContextPose, PositionWrapper as ContextPosition,
                                   ConfigurationWrapper as ContextConfiguration, addNumpyYaml)


class Box(yaml.YAMLObject):
    yaml_tag = u'!Box'

    def __init__(self, width, length, height):
        self.width = width
        self.length = length
        self.height = height

    def __repr__(self):
        return "%s(width=%r, length=%r, height=%r)" % (self.__class__.__name__, self.width, self.length, self.height)


class Sphere(yaml.YAMLObject):
    yaml_tag = u'!Sphere'

    def __init__(self, radius):
        self.radius = radius

    def __repr__(self):
        return "%s(radius=%r)" % (self.__class__.__name__, self.radius)


class Mesh(yaml.YAMLObject):
    yaml_tag = u'!Mesh'

    def __init__(self, meshFile, pose):
        self.meshFile = meshFile
        self.pose = pose

    def __repr__(self):
        return "%s(meshFile=%r, pose=%r)" % (self.__class__.__name__, self.meshFile, self.pose)


class MoveItWrapper(PortfolioMethod, ArmPlanner, ArmController):
    """ A wrapper class for MoveIt that fulfills the ArmPlanner and ArmController interfaces. """
    def __init__(self, parameters):
        addNumpyYaml()
        self._basePath = ''
        self._loadModelPaths(ROSUtils.resolvePath(parameters['modelPathFile']))
        self._moveItLaunchFile = ROSUtils.resolvePath(parameters['moveItLaunchFile'])
        moveit_commander.roscpp_initialize(sys.argv)
        self._moveit_commander = moveit_commander.RobotCommander()
        self._moveit_scene = moveit_commander.PlanningSceneInterface()
        self._moveGroups = {}
        # set up yaml to be able to read Poses (contain numpy arrays)

    def getName(self):
        return "MoveItWrapper"

    def getParameters(self, role, paramPrefix):
        if role == 'ArmPlanner':
            # paramPrefix + 'algorithm': ('categorical', ['SBLkConfigDefault','LBKPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'LazyRRTkConfigDefault', 'ESTkConfigDefault', 'KPIECEkConfigDefault', 'RRTStarkConfigDefault', 'BKPIECEkConfigDefault'], 'RRTkConfigDefault')}
            return {paramPrefix + '_moveGroup': ('categorical', ['left_arm', 'right_arm'], 'left_arm'),
                    paramPrefix + '_algorithm': ('categorical', ['LBKPIECEkConfigDefault', 'RRTkConfigDefault', 'RRTConnectkConfigDefault', 'RRTStarkConfigDefault'], 'RRTkConfigDefault'),
                    paramPrefix + '_range': ('real', [0.0, 100.0], 0.0),
                    paramPrefix + '_timeout': ('real', [4.0, 15.0], 5.0),
                    paramPrefix + '_longest_valid_segment_fraction': ('real', [0.001, 1.0], 0.01),
                    paramPrefix + '_goal_bias': ('real', [0.0, 1.0], 0.05),
                    paramPrefix + '_border_fraction': ('real', [0.0, 1.0], 0.9),
                    paramPrefix + '_min_valid_path_fraction': ('real', [0.0, 1.0], 0.5),
                    paramPrefix + '_failed_expansion_score_factor': ('real', [0.00001, 1.0], 0.5),
                    paramPrefix + '_delay_collision_checking': ('categorical', [0, 1], 1)}
        return {}

    def getConditionals(self, role, paramPrefix):
        if role == 'ArmPlanner':
            return {paramPrefix + '_goal_bias': paramPrefix + '_goal_bias | ' + paramPrefix + '_algorithm in [RRTkConfigDefault, RRTStarkConfigDefault]',
                    paramPrefix + '_border_fraction': paramPrefix + '_border_fraction' + ' | ' + paramPrefix + 'algorithm' + '_algorithm in [LBKPIECEkConfigDefault, KPIECE, BKPIECE]',
                    paramPrefix + '_min_valid_path_fraction': paramPrefix + '_min_valid_path_fraction | ' + paramPrefix + '_algorithm in [LBKPIECEkConfigDefault, KPIECE, BKPIECE]',
                    paramPrefix + '_failed_expansion_score_factor': paramPrefix + '_failed_expansion_score_factor | ' + paramPrefix + '_algorithm in [KPIECE, BKPIECE]',
                    paramPrefix + '_delay_collision_checking': paramPrefix + '_delay_collision_checking | ' + paramPrefix + '_algorithm in [RRTStarkConfigDefault]'}
        return {}

    def getForbiddenConfigurations(self, role):
        # TODO
        return None

    def initialize(self):
        # We start MoveIt here using the launch file specified
        MoveItUtils.runMoveIt(self._moveItLaunchFile, keepRunning=True)

    def allocateResources(self, roles=None):
        pass

    def releaseResources(self, roles=None):
        pass

    def hasResourceConflict(self, activeRoles):
        return False

    def destroy(self):
        MoveItUtils.runMoveIt(keepRunning=False)
        self.releaseResources()

    def supportsBatchProcessing(self):
        return False

    def preparePlanning(self, context):
        # TODO decide whether init should count in to runtime
        self._synchEnvironment(context)

    def plan(self, goal, context, paramPrefix, parameters):
        self.preparePlanning(context)
        moveGroupName = parameters[paramPrefix + '_moveGroup']
        algorithmName = parameters[paramPrefix + '_algorithm']
        timeOut = parameters[paramPrefix + '_timeout']
        longest_valid_segment_fraction = parameters[paramPrefix + '_longest_valid_segment_fraction']
        if moveGroupName not in self._moveGroups:
            self._moveGroups[moveGroupName] = moveit_commander.MoveGroupCommander(moveGroupName)
        moveGroup = self._moveGroups[moveGroupName]
        moveGroup.set_planning_time(timeOut)
        self._setMoveItROSParameters(paramPrefix, parameters)
        moveGroup.clear_pose_targets()
        robotState = self._convertRobotState(context, moveGroup.get_active_joints())
        moveGroup.set_start_state(robotState)
        if isinstance(goal, ContextPose):
            # input goal is pose, transform to ROSPose and send it to Moveit
            pose = ROSUtils.contextPoseToROSPose(goal, bStamped=True)
            moveGroup.set_pose_target(pose)
        elif isinstance(goal, ContextPosition):
            # input is just a position, transform to list and send it to Moveit
            position = [x for x in goal.position]
            moveGroup.set_position_target(position)
        elif isinstance(goal, ContextConfiguration):
            # if input goal is configuration, plan to config
            moveGroup.set_joint_value_target(goal.configuration)
        else:
            raise ValueError('The given goal %g is of invalid type.' % goal)

        # TODO extend so that contraints are respected (e.g. follow cartesian path)
        moveGroup.set_planner_id(algorithmName)
        moveit_traj = moveGroup.plan()
        if len(moveit_traj.joint_trajectory.points) == 0:
            # we did not find a solution
            return None
        # else extract the solution
        trajectory = Trajectory(group_name=parameters[paramPrefix + '_moveGroup'],
                                joint_names=moveit_traj.joint_trajectory.joint_names)
        for point in moveit_traj.joint_trajectory.points:
            wp = Waypoint(timestamp=(point.time_from_start.secs, point.time_from_start.nsecs),
                          positions=point.positions, velocities=point.velocities,
                          accelerations=point.accelerations)
            trajectory.appendWaypoint(wp)
        return trajectory

    def execute(self, trajectory, context, paramPrefix, parameters):
        group = self._moveit_commander.get_group(trajectory.group_name)
        moveit_traj = self._convertTrajectory(trajectory)
        executionResult = group.execute(moveit_traj)
        return executionResult

    def _loadModelPaths(self, modelPathFile):
        f = open(modelPathFile, 'r')
        fileContent = f.read()
        if fileContent == "":
            f.close()
            raise IOError('The given model path file %s is empty.' % modelPathFile)
        modelInfo = yaml.load(fileContent)
        base_path = ""
        # There is either a base path specified in the file
        if 'base_path' in modelInfo:
            base_path = modelInfo['base_path']
            # It might contain $ROS_PACKAGE_PATH macros, so let's resolve these
            base_path = ROSUtils.resolvePath(base_path)
        # or there is no base path and we assume that we are given absolute paths for each object file
        self._basePath = base_path
        self._modelPaths = modelInfo['models']
        f.close()

    def _synchEnvironment(self, context):
        si = context.getSceneInformation()
        # First clear the world
        self._moveit_scene.remove_world_object()
        # now repopulate
        for obj in si.objects:
            objModel = self._modelPaths[obj.name]
            pose = ROSUtils.contextPoseToROSPose(obj.pose, bStamped=True)
            if isinstance(objModel, Box):
                self._moveit_scene.add_box(obj.name, pose,
                                           (objModel.width, objModel.length, objModel.height))
            elif isinstance(objModel, Sphere):
                self._moveit_scene.add_sphere(obj.name, pose,
                                              objModel.radius)
            elif isinstance(objModel, Mesh):  # it has to be a Mesh -> get path to mesh + transform
                if self._basePath == '':
                    meshFile = ROSUtils.resolvePath(objModel.meshFile)
                else:
                    meshFile = self._basePath + objModel.meshFile
                # We have to compute the transform from the mesh frame to world frame
                eulerAngles = tf.transformations.euler_from_quaternion(objModel.pose.orientation)
                meshToObject = tf.transformations.compose_matrix(translate=objModel.pose.position, angles=eulerAngles)
                eulerAngles = tf.transformations.euler_from_quaternion(obj.pose.orientation)
                objectToWorld = tf.transformations.compose_matrix(translate=obj.pose.position, angles=eulerAngles)
                meshToWorld = numpy.dot(objectToWorld, meshToObject)
                orientation = tf.transformations.quaternion_from_matrix(meshToWorld)
                position = tf.transformations.translation_from_matrix(meshToWorld)
                pose = ROSUtils.contextPoseToROSPose(ContextPose(position=position, orientation=orientation),
                                                     bStamped=True)
                # print 'Publising object %s at pose %s with mesh %s to MoveIt!' % (obj.name, pose, meshFile)
                self._moveit_scene.add_mesh(obj.name, pose, meshFile)
            else:
                raise RuntimeError('Unknown object model type: %s' % objModel)

    def _convertRobotState(self, context, jointStates):
        si = context.getSceneInformation()
        robotInfo = si.getRobotInfo()
        config = robotInfo.configuration
        robotStateMsg = RobotState()
        jointStateMsg = JointState(header=Header(stamp=rospy.Time.now(), frame_id='/world'))
        for jointName in jointStates:
            jointStateMsg.name.append(jointName)
            jointStateMsg.position.append(config[jointName])
            # jointStateMsg.velocity.append(0.0)
            # jointStateMsg.effort.append(0.0)
        robotStateMsg.joint_state = jointStateMsg
        return robotStateMsg

    def _convertTrajectory(self, trajectory):
        points = []
        for wp in trajectory.waypoints:
            timeStamp = rostime.Duration(secs=wp.timestamp[0], nsecs=wp.timestamp[1])
            jtp = JointTrajectoryPoint(positions=wp.positions, velocities=wp.velocities,
                                       accelerations=wp.accelerations, time_from_start=timeStamp)
            points.append(jtp)
        jointTraj = JointTrajectory(joint_names=trajectory.joint_names, points=points)
        robotTraj = RobotTrajectory()
        robotTraj.joint_trajectory = jointTraj
        return robotTraj

    def _setMoveItROSParameters(self, paramPrefix, parameters):
        moveGroupName = parameters[paramPrefix + '_moveGroup']
        value = parameters[paramPrefix + '_longest_valid_segment_fraction']
        rospy.set_param('/move_group/' + moveGroupName + '/longest_valid_segment_fraction', value)
        algoName = parameters[paramPrefix + '_algorithm']
        if algoName == 'RRTkConfigDefault':
            rospy.set_param('/move_group/planner_configs/RRTkConfigDefault/range', parameters[paramPrefix + '_range'])
            rospy.set_param('/move_group/planner_configs/RRTkConfigDefault/goal_bias', parameters[paramPrefix + '_goal_bias'])
        elif algoName == 'LBKPIECEkConfigDefault':
            rospy.set_param('/move_group/planner_configs/LBKPIECEkConfigDefault/range', parameters[paramPrefix + '_range'])
            rospy.set_param('/move_group/planner_configs/LBKPIECEkConfigDefault/border_fraction', parameters[paramPrefix + '_border_fraction'])
            rospy.set_param('/move_group/planner_configs/LBKPIECEkConfigDefault/min_valid_path_fraction', parameters[paramPrefix + '_min_valid_path_fraction'])
        elif algoName == 'RRTConnectkConfigDefault':
            rospy.set_param('/move_group/planner_configs/RRTConnectkConfigDefault/range', parameters[paramPrefix + '_range'])
        elif algoName == 'RRTStarkConfigDefault':
            rospy.set_param('/move_group/planner_configs/RRTStarkConfigDefault/range', parameters[paramPrefix + '_range'])
            rospy.set_param('/move_group/planner_configs/RRTStarkConfigDefault/goal_bias', parameters[paramPrefix + '_goal_bias'])
            rospy.set_param('/move_group/planner_configs/RRTStarkConfigDefault/delay_collision_checking', parameters[paramPrefix + '_delay_collision_checking'])
