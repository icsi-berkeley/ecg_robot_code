"""
Author: seantrott <seantrott@icsi.berkeley.edu>

------
See LICENSE.txt for licensing information.
------

"""

from robots.agent_solver import AgentSolver
from robots.morse_solver import MorseRobotProblemSolver
import sys
import json

class MorseAgentSolver(AgentSolver, MorseRobotProblemSolver):
    def __init__(self, args):
        AgentSolver.__init__(self, args)
        MorseRobotProblemSolver.__init__(self, args)

if __name__ == "__main__":
    boss = MorseAgentSolver(sys.argv[1:])