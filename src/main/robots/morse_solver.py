"""
A Morse Problem Solver (extends BasicRobotProblemSolver).
Author: seantrott <seantrott@icsi.berkeley.edu>

"""

# TODO
from robot_solver_alt import *
from robot_utils.avoidance import TwoDimensionalAvoidanceSolver

class MorseRobotProblemSolver(BasicRobotProblemSolver, TwoDimensionalAvoidanceSolver):
    def __init__(self, args):
        BasicRobotProblemSolver.__init__(self, args)
        TwoDimensionalAvoidanceSolver.__init__(self)
        self.world = build('morse')
        #self.build_world(False)

    def build_world(self, update=False):
        """ This automatically populates the world model with objects from the Morse scene,
        which aren't necessarily in the builder.py file. UPDATE flag can be changed in __init__
        to execute it or not. """
        if update:
            robot1 = self.world.robot1_instance
            self.update_world(agent=robot1, discovered=[])

    def move(self, mover, x, y, z=0.0, speed=2, tolerance=3.5, collide=False):
        if collide:
            new, interrupted = mover.move_np(x=x, y=y, z=z, speed=speed, tolerance=tolerance)
        else:
            origin = [mover.pos.x, mover.pos.y]
            destination = [x, y]  #z?
            line = self.compute_line(origin, destination, mover)
            smoothed = self.smooth_trajectory(line)
            for point in smoothed:
                new, interrupted = mover.move(x=point[0], y=point[1], z=0, speed=speed, tolerance=tolerance)
                if interrupted:
                    self.update_world(agent=mover, discovered=new)
                    self.move(mover, x, y, z, speed, tolerance, collide=False)
        self.update_world(agent=mover)


    def update_world(self, agent, discovered=[]):
        newworld = agent.get_world_info()
        # This will need to be changed dependent on worldview; 
        for obj in newworld:
            if hasattr(self.world, obj['name']):    
                setattr(getattr(self.world, obj['name']), 'pos',Struct(x =obj['position'][0], y=obj['position'][1], z =obj['position'][2]) )
            else:
                #if obj['type'] in discovered:
                if obj['name'] in discovered:
                    description = json.loads(obj['description'])
                    self.world.__dict__[obj['name']] = Struct(pos=Struct(x =obj['position'][0], y=obj['position'][1], z = obj['position'][2]), 
                                                              name=obj['name'],
                                                              type=obj['type'],
                                                              color=description['color'],
                                                              size=description['size'])
                    msg = "I discovered a {} at position ({}, {}).".format(obj['type'], obj['position'][0], obj['position'][1])
                    self.respond_to_query(message=msg)

    def getpos(self, inst):
        instance =getattr(self.world, inst)
        p = instance.pos
        return (p.x, p.y, p.z) 





if __name__ == "__main__":
    solver = MorseRobotProblemSolver(sys.argv[1:])