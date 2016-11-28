"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains an implementation of the method type 'GraspController' as defined in deliverable D4.1.
    It uses the baxter robot

    @author: Silvia Cruciani (cruciani@kth.se)
"""
from manipulation_dreambed.MethodTypes import GraspController
import sys
import rospy
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION


class SimpleGraspController(GraspController):
    def __init__(self, parameters):
        self._left = baxter_interface.Gripper('left', CHECK_VERSION)
        self._right = baxter_interface.Gripper('right', CHECK_VERSION)

    def getName(self):
        """ Returns the name of this method. """
        return "SimpleGraspController"

    def initialize(self):
        self._left.calibrate()
        self._right.calibrate()
        self._left.open()
        self._right.open()

    def allocateResources(self):
        pass

    def releaseResources(self):
        pass

    def getParameters(self):
        """ Returns a pySMAC compatible parameter definition (see deliverable D2.1) """
        return {}

    def getConditionals(self):
        """ Returns a pySMAC compatible definition of conditional parameters (see deliverable D2.1) """
        pass

    def getForbiddenConfigurations(self):
        """ Returns a pySMAC compatible definition of forbidden parameters (see deliverable D2.1) """
        pass

    def startExecution(self, grasp, context, paramPrefix, parameters):
        """Only close the grippers? lalalala git this file is different you see?"""
        self._left.close()
        self._rigth.close()
        pass

    def stopExecution(self):
        pass

    def destroy(self):
        pass
