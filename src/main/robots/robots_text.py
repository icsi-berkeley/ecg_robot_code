"""
Subclasses text_agent. 

------
See LICENSE.txt for licensing information.
------
"""

from nluas.language.text_agent import *


class RobotTextAgent(TextAgent):
	def __init__(self, args):
		TextAgent.__init__(self, args)


if __name__ == "__main__":
	text = RobotTextAgent(sys.argv[1:])
	while True:
		text.prompt()