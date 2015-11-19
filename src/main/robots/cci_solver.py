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