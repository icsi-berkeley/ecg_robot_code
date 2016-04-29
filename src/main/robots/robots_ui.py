"""
Author: seantrott <seantrott@icsi.berkeley.edu>
Similar to a regular UserAgent, but it uses a RobotSpecializer instead.

------
See LICENSE.txt for licensing information.
------
"""

from nluas.language.user_agent import *
from robots.robot_specializer import *
import sys
import subprocess

class RobotUserAgent(UserAgent):
	def __init__(self, args):
		UserAgent.__init__(self, args)

	def initialize_specializer(self):
		self.specializer=RobotSpecializer(self.analyzer)

	def output_stream(self, tag, message):
		print("{}: {}".format(tag, message))
		# MAC only
		#subprocess.Popen(["say", message])


if __name__ == "__main__":
	ui = RobotUserAgent(sys.argv[1:])
	ui.prompt()
