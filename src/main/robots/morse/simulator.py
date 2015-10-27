"""Created on Mar 6, 2014 by @author lucag.
"""

from pymorse import Morse  
from nluas.utils import update, Struct
from pprint import pprint


#from builder import build

def print_pos(pose):
    print("I'm currently at %s" % pose)


class Box(object):
    """A simple box object that knows where it is 
    """
    def __init__(self, name, type, pos, color, size):
        update(self, name=name, type = type, pos=pos, color=color, size = size,simulator=Morse())
        #print(self.simulator.__dict__)
        #print ('\n')
      #  inst = getattr(self.simulator, self.name)
        #inst.pose.subscribe(self.setpos)
    
    def setpos(self, pos):
        self.pos = Struct(**pos)


class Robot(object):
    """A simple controller for a Morse simulated robot.
    """
    def __init__(self, name, size=1):
        update(self, name=name, pos=Struct(x=0.0, y=0.0, z=0.0), simulator=Morse(), type="robot", size = size)
        q = "pose_"
        q += self.name
        inst = getattr(self.simulator, self.name)
        pose = getattr(inst, q)
        pose.subscribe(self.setpos)

    def activate_attribute(self, attribute):
        a = self.name + "." + self.get_attribute(attribute)

        self.simulator.rpc("simulation", "activate", a)

    def deactivate_attribute(self, attribute):
        a = self.name + "." + self.get_attribute(attribute)
        self.simulator.rpc("simulation", "deactivate", a)

    def get_attribute(self, attribute):
        q = attribute + "_"
        q += self.name
        return q
        #inst = getattr(self.simulator, self.name)
        #return getattr(inst, q)

    def get_pos(self):
        return self.pose

    def setpos(self, pos):
        self.pos = Struct(**pos)

    def stop(self):
        inst = getattr(self.simulator, self.name)
        q = "motion_"
        q += self.name
        motion = getattr(inst, q)
        motion.stop()


    def move_together(self, **to):
        self.deactivate_attribute("orientation")
        self.activate_attribute("motion")

        inst = getattr(self.simulator, self.name)

        q = "motion_"
        q += self.name
        motion = getattr(inst, q)
        #print(motion.get_status().result())
        motion.publish(to)
        #p = inst.proximity
        #inst.motion.goto(to['x'],to['y'] ,to['tolerance'],to['speed'])
        #print(inst.motion.get_configurations())
        # Leave a couple of ms to the simulator to start the action.
        self.simulator.sleep(0.3)
        return True



    def move_np(self, **to):
        """ Moves without proximity sensor."""
        self.deactivate_attribute("orientation")
        self.activate_attribute("motion")

        inst = getattr(self.simulator, self.name)
        q = "motion_"
        q += self.name
        motion = getattr(inst, q)
        #print(motion.get_status().result())
        motion.publish(to)
        self.simulator.sleep(0.3)
    
        # waits until we reach the target
        discovered = []
        interrupted=False
        
        while motion.get_status().result() != "Arrived":
            pass
            #self.simulator.sleep(0.2) 
        
        return discovered, interrupted


    def move(self, **to):
        self.deactivate_attribute("orientation")
        self.activate_attribute("motion")

        inst = getattr(self.simulator, self.name)
        q = "motion_"
        q += self.name
        motion = getattr(inst, q)

        motion.publish(to)
        qprox = "proximity_"
        qprox += self.name
        p = getattr(inst, qprox)
        #p = inst.proximity
        #inst.motion.goto(to['x'],to['y'] ,to['tolerance'],to['speed'])
        #print(inst.motion.get_configurations())
        # Leave a couple of ms to the simulator to start the action.
        self.simulator.sleep(0.5)
    
        # waits until we reach the target
        discovered = []
        interrupted=False
        
        while motion.get_status() != "Arrived":
            prox = p.get()['near_objects'].keys()
            if len(prox) > 0:
                for obj in p.get()['near_objects'].keys():
                    discovered.append(obj)
                interrupted=True
                return (discovered, interrupted)
            pass
        return discovered, interrupted

    def get_camera(self):
        inst = getattr(self.simulator, self.name)
        q = "camera_"
        q += self.name
        return getattr(inst, q)

       
    def get_world_info(self):
        inst = getattr(self.simulator, self.name)
        q = "camera_"
        q += self.name
        camera = getattr(inst, q)
        return self.get_camera().get()['visible_objects']
         #update the model of the wold (based on damages you did...)

    def close(self):
        self.simulator.quit()


    def rotate(self, **args):
        self.deactivate_attribute("motion")
        self.activate_attribute("orientation")
        q = "orientation_"
        q += self.name
        inst = getattr(self.simulator, self.name)
        orientation = getattr(inst, q)
        orientation.publish(args)
    
        

