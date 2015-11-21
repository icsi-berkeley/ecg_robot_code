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


"""


from robot_solver import *

class CCIProblemSolver(BasicRobotProblemSolver):
	def __init__(self, args):
		BasicRobotProblemSolver.__init__(self, args)


	def move(self, agent, x, y, z=0.0, speed=2, tolerance=3.5, collide=False):
		commandInfo = {'commandName': 'moveToXY',
					   'commArgs': [x, y]}
		return commandInfo
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