"""
Inherits from QueueSolver, puts commands on a Queue to be solved.

"""

from robots.robot_solver import BasicRobotProblemSolver
from robots.queue_solver import QueueSolver
from robots.morse_solver import MorseRobotProblemSolver
from robots.morse.xnet_simulator import Robot
#from robots.morse.xnet_sim2 import Robot
import sys

class XnetRobotSolver(BasicRobotProblemSolver, QueueSolver):
    """Combines BasicRobotProblemSolver and QueueSolver in a single class. """
    def __init__(self, args):
        QueueSolver.__init__(self, args)
        BasicRobotProblemSolver.__init__(self, args)
        self.build_solver_processor(self.commandQ, self, self.xnet_worker)
        self.world = self.build_world('morse/world.json')


class XnetMorseSolver(MorseRobotProblemSolver, QueueSolver):#, XnetRobotSolver):
    def __init__(self, args):
        #print("Initializing the XnetMorseSolver")
        #XnetRobotSolver.__init__(self, args)
        MorseRobotProblemSolver.__init__(self, args)
        QueueSolver.__init__(self, args)
        self.xnet_worker = getattr(self.world, "robot1_instance")
        self.build_solver_processor(self.commandQ, self, self.xnet_worker)



    def build_world(self, external_file):
        world = BasicRobotProblemSolver.build_world(self, external_file)
        robot1_instance=Robot('robot1_instance')
        setattr(world, 'robot1_instance', robot1_instance)
        #print(world)
        return world


    def move(self, mover, x, y, z=0.0, speed=2, tolerance=3.5, collide=False):
        kwargs={'x':x, 'y':y, 'z':z, 'tolerance': tolerance, 'speed': speed, 'collide': collide}
##        print('solver.move kwargs: ', kwargs)
        if not mover.isStarted(): 
           mover.build_move_xnet()
           mover.update_motion(**kwargs)
           mover.start()
           #print('move started')
        elif mover.isDone():
##           print('move is done')  
           mover.build_move_xnet()
           mover.update_motion(**kwargs)
           mover.start()
        elif mover.isOngoing():
##           print('about to stop move Xnet') 
           mover.suspend()   
           mover.update_motion(**kwargs)
           mover.restart()
##        print('exiting xnet solver move')
        self.update_world(agent=mover)


    def stop(self, mover):
##        print('xnet solver stop requested')
        self.solver_command_processor.set_suspended_expected(True)
        mover.suspend() 

    def resume(self, mover):
        self.solver_command_processor.set_continue_expected(False)        
        mover.resume()

if __name__ == "__main__":
    solver = XnetMorseSolver(sys.argv[1:])
    #solver = XnetRobotSolver(sys.argv[1:])
