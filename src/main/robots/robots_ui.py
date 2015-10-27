"""
Author: seantrott <seantrott@icsi.berkeley.edu>
Similar to a regular UserAgent, but it uses a RobotSpecializer instead.
"""

from nluas.language.user_agent import *
from robot_specializer import *
import sys

class RobotUserAgent(UserAgent):
	def __init__(self, args):
		UserAgent.__init__(self, args)

	def initialize_specializer(self):
		self.specializer=RobotSpecializer(self.analyzer)


if __name__ == "__main__":
	ui = RobotUserAgent(sys.argv[1:])
	ui.prompt()
