import rospy
from baxter_core_msgs.msg import AssemblyState
import baxter_interface
from controller_manager_msgs.srv import ReloadControllerLibraries, ListControllers, LoadController, SwitchController, SwitchControllerRequest

class BaxterSimulator(RobotSimulator):
    def __init__(self):
        self.controllerProcess = None
        self.bootedUp = False
        self.controllerResetService = None
        self.listControllersService = None
        self.loadControllerService = None
        self.switchControllerService = None
        self.baxter = None
        self.eStopPublisher = None

    """ The baxter simulator needs some time to boot up. We can not continue
        doing anything before the robot is booted. Therefore, this class listens
        to the topic /robot/state and blocks until a first message is received. Next,
        it starts the ros_control controllers of baxter. """
    def init(self):
        self.bootedUp = False
        # in case the simulator has not been started yet, we can receive the sim started message
        rospy.Subscriber('/robot/sim/started', EmptyMessage, self.callback)
        # else it might already be running, if everything is working, we should then get a robot state
        rospy.Subscriber('/robot/state', AssemblyState, self.callback)
        # self.eStopPublisher = rospy.Publisher('/robot/eStopTopic', BoolMessage, latch=True, queue_size=1)
        self.resetHardwarePublisher = rospy.Publisher('/robot/reset_hardware', EmptyMessage, queue_size=1)
        rospy.loginfo('GazeboSimulatorWrapper: Waiting for Baxter to come up...')
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
            self.reset()
        rospy.loginfo('GazeboSimulatorWrapper: Baxter is up, simulator is ready.')
        return self.bootedUp

    def reset(self):
        if self.controllerProcess is not None:
            self.controllerProcess.stop()
            self.controllerProcess = None
        # package = 'baxter_interface'
        # executable = 'joint_trajectory_action_server.py'
        # node = roslaunch.core.Node(package, executable)
        # launch = roslaunch.scriptapi.ROSLaunch()
        # launch.start()
        # self.controllerProcess = launch.launch(node)
        if self.baxter is None:
            rospy.error('GazeboSimulatorWrapper: Baxter simulator is not set, did you call init()?')
        self.baxter.stop()
        # self.eStopPublisher.publish(data=True)
        # self.eStopPublisher.publish(data=False)
        # self._reloadControllers()
        self._resetHardware()
        self.baxter.reset()
        self.baxter.enable()
        rospy.loginfo('GazeboSimulatorWrapper: Baxter successfully reset.')

    def _resetHardware(self):
        if self.resetHardwarePublisher is not None:
            self.resetHardwarePublisher.publish()
        else:
            raise RuntimeError('GazeboSimulatorWrapper: Could not reset hardware because no publisher set. Did you initialize?')

    def _reloadControllers(self):
        originalControllers = self.listControllersService()
        resetResult = self.controllerResetService(True)
        if not resetResult.ok:
            rospy.logerror('GazeboSimulatorWrapper: Could not reset Baxter controllers.')
            raise RuntimeError('GazeboSimulatorWrapper: Could not reload controllers.')
        for controller in originalControllers.controller:
            self.loadControllerService(controller.name)
            if controller.name == 'joint_state_controller':
                self.switchControllerService(start_controllers=[controller.name],
                                             stop_controllers=[],
                                             strictness=SwitchControllerRequest.BEST_EFFORT)
            # TODO: see if we need to set the controller states also

    def callback(self, data):
        self.bootedUp = True

    def cleanUp(self):
        self.baxter.disable()
        if self.controllerProcess is not None:
            self.controllerProcess.stop()
            self.controllerProcess = None


