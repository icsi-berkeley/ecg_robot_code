"""
Inherits from QueueSolver, puts commands on a Queue to be solved.

"""

from robots.robot_solver import BasicRobotProblemSolver
from robots.queue_solver import QueueSolver
from robots.morse_solver import MorseRobotProblemSolver
from robots.morse.xnet_simulator import Robot
import sys

class XnetRobotSolver(BasicRobotProblemSolver, QueueSolver):
    """Combines BasicRobotProblemSolver and QueueSolver in a single class. """
    def __init__(self, args):
        QueueSolver.__init__(self, args)
        BasicRobotProblemSolver.__init__(self, args)
        self.build_solver_processor(self.commandQ, self, self.xnet_worker)
        self.world = self.build_world('morse/world.json')


class XnetMorseSolver(MorseRobotProblemSolver, XnetRobotSolver):
    def __init__(self, args):
        XnetRobotSolver.__init__(self, args)
        MorseRobotProblemSolver.__init__(self, args)
        inst = getattr(self.world, "robot1_instance")
        self.build_solver_processor(self.commandQ, self, inst)



    def build_world(self, external_file):
        world = BasicRobotProblemSolver.build_world(self, external_file)
        robot1_instance=Robot('robot1_instance')
        setattr(world, 'robot1_instance', robot1_instance)
        #print(world)
        return world


if __name__ == "__main__":
    solver = XnetMorseSolver(sys.argv[1:])
    #solver = XnetRobotSolver(sys.argv[1:])
