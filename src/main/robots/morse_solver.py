"""
A Morse Problem Solver (extends BasicRobotProblemSolver).
Author: seantrott <seantrott@icsi.berkeley.edu>

"""

# TODO
from robot_solver import *
from robot_utils.avoidance import TwoDimensionalAvoidanceSolver
from morse.simulator import Robot

class MorseRobotProblemSolver(BasicRobotProblemSolver, TwoDimensionalAvoidanceSolver):
    def __init__(self, args):
        BasicRobotProblemSolver.__init__(self, args)
        TwoDimensionalAvoidanceSolver.__init__(self)
        self.world = self.build_world('morse/world.json')
        #self.build_world(False)


    def build_world(self, external_file):
        world = BasicRobotProblemSolver.build_world(self, external_file)
        robot1_instance=Robot('robot1_instance')
        robot2_instance=Robot('robot2_instance')
        setattr(world, 'robot1_instance', robot1_instance)
        setattr(world, 'robot2_instance', robot2_instance)
        #print(world)
        return world


    def move(self, mover, x, y, z=0.0, speed=2, tolerance=3.5, collide=False):
        if collide:
            new, interrupted = mover.move_np(x=x, y=y, z=z, speed=speed, tolerance=tolerance)
        else:
            origin = [mover.pos['x'], mover.pos['y']]
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
                setattr(getattr(self.world, obj['name']), 'pos',dict(x =obj['position'][0], y=obj['position'][1], z =obj['position'][2]) )
            else:
                #if obj['type'] in discovered:
                if obj['name'] in discovered:
                    description = json.loads(obj['description'])
                    self.world.__dict__[obj['name']] = Struct(pos=Struct(x =obj['position'][0], y=obj['position'][1], z = obj['position'][2]), 
                                                              name=obj['name'],
                                                              type=obj['type'],
                                                              color=description['color'],
                                                              size=description['size'],
                                                              weight=description['weight'])
                    msg = "I discovered a {} at position ({}, {}).".format(obj['type'], obj['position'][0], obj['position'][1])
                    self.respond_to_query(message=msg)

    def getpos(self, inst):
        instance =getattr(self.world, inst)
        p = instance.pos
        return (p['x'], p['y'], p['z']) 





if __name__ == "__main__":
    solver = MorseRobotProblemSolver(sys.argv[1:])