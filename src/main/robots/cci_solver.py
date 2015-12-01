"""
This is a Problem Solver extended from the BasicRobotProblemSolver,
which makes API calls to CCI, an interface designed to communicate
with ROS systems (including Darwin and NAO). Ideally, this should
nicely modularize the solver and reduce redundancies in the code.

author contact: seantrott@icsi.berkeley.edu

Initial goals:
-move to an (x, y) coordinate
-build/update a world model
-move to an object
-grasp an object

This should "publish" commands to ROS/CCI.


"""
import rospy

import inspect
from std_msgs.msg import *
from gazebo_msgs.msg import ModelStates,LinkStates

from robots.robot_solver import *

class CCIProblemSolver(BasicRobotProblemSolver):
    def __init__(self, args):
        BasicRobotProblemSolver.__init__(self, args)
        self.publisher = rospy.Publisher('/cqi/command', String, queue_size=5)
        rospy.sleep(1)

    def publish(self, commandName, commArgs):
        # Bit hacky, need to check that this works for non-move commands...
        command = "{}({})".format(commandName, str(commArgs).replace("[", "").replace("]", ""))
        rospy.loginfo("Publishing CQI command " + command)
        self.publisher.publish(String(command))
        # This will publish to the ROS interface


    def move(self, agent, x, y, z=0.0, speed=2, tolerance=3.5, collide=False):
        self.publish('moveToXY', [x, y])
        # TO DO: makes API call to CCI/ROS-interface, instructs AGENT to move to coordinate (x, y)

    def moveToPose(self, agent, x, y, rotation):
        pass
        # TO DO: makes API call to CCI/ROS-interface, instructs AGENT to move to coordinate (x, y) at angle ROTATION

    def grasp(self, agent, obj):
        pass
        # TO DO: makes API call to CCI/Ros-interface, instructs AGENT to grasp object_name
        # Note: "command_grasp" will move to object location, then call this method


if __name__ == "__main__":
    solver = CCIProblemSolver(sys.argv[1:])
    rospy.loginfo("Starting NLU system...")
    rospy.init_node("nlu_solver")


