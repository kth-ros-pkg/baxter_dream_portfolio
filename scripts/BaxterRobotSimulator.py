#!/usr/bin/env python
import rospy
import time
import baxter_interface
import roslaunch
from std_srvs.srv import Empty as EmptyService
from std_srvs.srv import Trigger as TriggerService
from std_srvs.srv import TriggerResponse
from std_srvs.srv import EmptyResponse
from std_msgs.msg import Empty as EmptyMessage
from baxter_core_msgs.msg import AssemblyState
from controller_manager_msgs.srv import ReloadControllerLibraries, ListControllers, LoadController, SwitchController, SwitchControllerRequest

class BaxterSimulator(object):
    def __init__(self):
        self.controllerProcess = None
        self.bootedUp = False
        self.controllerResetService = None
        self.listControllersService = None
        self.loadControllerService = None
        self.switchControllerService = None
        self.baxter = None
        self.eStopPublisher = None


    def init(self, empty_msg):
        """ The baxter simulator needs some time to boot up. We can not continue
        doing anything before the robot is booted. Therefore, this class listens
        to the topic /robot/state and blocks until a first message is received. Next,
        it starts the ros_control controllers of baxter. """
        # if self.bootedUp:
            # return TriggerResponse(success=True, message='Baxter simulator already initialized')
        self.bootedUp = False
        # in case the simulator has not been started yet, we can receive the sim started message
        rospy.Subscriber('/robot/sim/started', EmptyMessage, self.callback)
        # else it might already be running, if everything is working, we should then get a robot state
        rospy.Subscriber('/robot/state', AssemblyState, self.callback)
        # self.eStopPublisher = rospy.Publisher('/robot/eStopTopic', BoolMessage, latch=True, queue_size=1)
        self.resetHardwarePublisher = rospy.Publisher('/robot/reset_hardware', EmptyMessage, queue_size=1)
        rospy.loginfo('BaxterSimulator: Waiting for Baxter to come up...')
        while not self.bootedUp and not rospy.is_shutdown():
            time.sleep(0.1)
        # wait for services
        rospy.wait_for_service('/robot/controller_manager/reload_controller_libraries')
        self.controllerResetService = rospy.ServiceProxy('/robot/controller_manager/reload_controller_libraries',
                                                         ReloadControllerLibraries)
        rospy.wait_for_service('/robot/controller_manager/list_controllers')
        self.listControllersService = rospy.ServiceProxy('/robot/controller_manager/list_controllers', ListControllers)
        rospy.wait_for_service('/robot/controller_manager/load_controller')
        self.loadControllerService = rospy.ServiceProxy('/robot/controller_manager/load_controller', LoadController)
        rospy.wait_for_service('/robot/controller_manager/switch_controller')
        self.switchControllerService = rospy.ServiceProxy('/robot/controller_manager/switch_controller',
                                                          SwitchController)

        if self.bootedUp:
            self.baxter = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
            self.reset(None)
        rospy.loginfo('BaxterSimulator: Baxter is up, simulator is ready.')
        return TriggerResponse(success=self.bootedUp, message='Baxter robot simulator init')

    def reset(self, empty_msg):
        # if self.controllerProcess is not None:
        #     self.controllerProcess.stop()
        #     self.controllerProcess = None
        # package = 'baxter_interface'
        # executable = 'joint_trajectory_action_server.py'
        # node = roslaunch.core.Node(package, executable)
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        # self.controllerProcess = launch.launch(node)
        if self.baxter is None:
            rospy.error('BaxterSimulator: Baxter simulator is not set, did you call init()?')
        self.baxter.stop()
        # self.eStopPublisher.publish(data=True)
        # self.eStopPublisher.publish(data=False)
        # self._reloadControllers()
        self._resetHardware()
        self.baxter.reset()
        self.baxter.enable()
        rospy.loginfo('BaxterSimulator: Baxter successfully reset.')
        return EmptyResponse()

    def _resetHardware(self):
        if self.resetHardwarePublisher is not None:
            self.resetHardwarePublisher.publish()
        else:
            raise RuntimeError('BaxterSimulator: Could not reset hardware because no publisher set. Did you initialize?')

    def _reloadControllers(self):
        originalControllers = self.listControllersService()
        resetResult = self.controllerResetService(True)
        if not resetResult.ok:
            rospy.logerror('BaxterSimulator: Could not reset Baxter controllers.')
            raise RuntimeError('BaxterSimulator: Could not reload controllers.')
        for controller in originalControllers.controller:
            self.loadControllerService(controller.name)
            if controller.name == 'joint_state_controller':
                self.switchControllerService(start_controllers=[controller.name],
                                             stop_controllers=[],
                                             strictness=SwitchControllerRequest.BEST_EFFORT)
            # TODO: see if we need to set the controller states also

    def callback(self, data):
        self.bootedUp = True

    def cleanUp(self, empty_msg):
        self.baxter.disable()
        if self.controllerProcess is not None:
            self.controllerProcess.stop()
            self.controllerProcess = None
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('BaxterRobotSimulator')
    baxter_simulator = BaxterSimulator()
    init_service = rospy.Service('/robot_simulator/init_robot_simulator', TriggerService,
                                 baxter_simulator.init)
    reset_service = rospy.Service('/robot_simulator/reset_robot_simulator', EmptyService,
                                  baxter_simulator.reset)
    cleanup_service = rospy.Service('/robot_simulator/cleanup_robot_simulator', EmptyService,
                                    baxter_simulator.cleanUp)
    rospy.loginfo('Running Baxter simulator wrapper.')
    rospy.spin()

