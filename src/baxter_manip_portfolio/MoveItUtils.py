"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains some utility functions for the use of MoveIt!.

    @author: Joshua Haustein (haustein@kth.se)
"""
import subprocess
import rospy


def static_vars(**kwargs):
    """ Adds the given arguments as attributes to the given function."""
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func
    return decorate


@static_vars(moveItProcess=None)
def runMoveIt(launchFile=None, keepRunning=True):
    if keepRunning:
        if runMoveIt.moveItProcess is None and launchFile is not None:
            rospy.loginfo('Starting MoveIt!')
            runMoveIt.moveItProcess = subprocess.Popen(['roslaunch', launchFile])
            rospy.loginfo('Sleeping for two seconds to wait for MoveIt to come up.')
            rospy.sleep(2.0)
            # creationflags=subprocess.CREATE_NEW_CONSOLE
        elif runMoveIt.moveItProcess is None:
            raise ValueError('Running MoveIt was requested, but no launch file is specified')
        else:
            rospy.loginfo('MoveIt keeps running')
    elif not keepRunning:
        if runMoveIt.moveItProcess is not None:
            rospy.loginfo('Stopping MoveIt')
            runMoveIt.moveItProcess.terminate()
            runMoveIt.moveItProcess = None
        else:
            rospy.loginfo('MoveIt is not running')
