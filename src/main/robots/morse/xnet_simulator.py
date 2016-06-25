from robots.morse.simulator import *
from robots.xnet_worker import *

class Robot(XnetWorker):
    """A simple controller for a Morse simulated robot.
    """

    def __init__(self, name, size=1, weight=2, fuel=100):
        XnetWorker.__init__(self)
        update(self, name=name, pos=Struct(x=0.0, y=0.0, z=0.0), simulator=Morse(), type="robot", size = size, weight=weight, fuel=fuel)
        self.inst = getattr(self.simulator, self.name)
        #self.build_move_xnet()
        self.jvmAttached = False

    def build_move_xnet(self):
        xnet = MoveXnet(self.inst)
        self.build_xnet(xnet)
        self.pendingMotion = None

    


    def get_pos():
        return  pose

    def setpos(self, pos):
        self.pos = Struct(**pos)
        #print(self.pos)


    def get_pending_motion(self):
        return self.pendingMotion

    def update_motion(self, **to):
##        print('simulator: updating motion')
        self.attach_jvm()
        self.pendingMotion = to 
##        to = self.get_pending_motion()
        self.get_xnet().targetLocation(to['x'],to['y'])
        self.get_xnet().setSpeed(to['speed'])
        self.get_xnet().setTolerance(to['tolerance'])
        self.get_xnet().setCollide(to['collide'])
        #print('simulator:  motion updated') 

##    def set_pending_motion(self, **to): 
##        self.pendingMotion = to 
##        #print('setPendingMotion ',to)


    # this isn't being called anymore, so we're not getting the proximity information? 
    def move(self, **to):
        #print('Simulator move: ')
        print(to)

        inst = getattr(self.simulator, self.name)

##        print('simulator move to: ',to)
        #TODO add logic for done / restart
        # if not started and first time, attach thread
        # if not started, or , set parms
        self.attach_jvm()
        #self.moveXnet = MoveXnet(inst)
        #self.moveXnet = self.getXnet()
##        self.set_pending_motion(**to) 
        self.update_motion(**to) 
        # updateMotion calls getPendingMotion  TODO
        #inst.motion.publish(to)  

        p = inst.proximity
        #print('proximity', p)
        #inst.motion.goto(to['x'],to['y'] ,to['tolerance'],to['speed'])
        #print(inst.motion.get_configurations())
        # Leave a couple of ms to the simulator to start the action.
        #print('simulator', self.simulator)
        #print('inst', inst)
        #self.simulator.sleep(0.1)
    
        # waits until we reach the target
        discovered = []
        interrupted=False
        self.simulator.sleep(.2)
        """
        while inst.motion.get_status() != "Arrived":
            prox = p.get()['near_objects'].keys()
            if len(prox) > 0:
                for obj in p.get()['near_objects'].keys():
                    discovered.append(obj)
                interrupted=True
                return (discovered, interrupted)

            self.simulator.sleep(0.2)
        """ 
        #print('discovered, interrupted',discovered,interrupted)
        return discovered, interrupted

    def move_collide(self, **to):
        inst = getattr(self.simulator, self.name)
        inst.motion_collide.publish(to)
        #inst.motion.goto(to['x'],to['y'] ,to['tolerance'],to['speed'])
        inst.motion._obstacle_avoidance = False
        #print(inst.motion._obstacle_avoidance )
        #print(inst.motion.get_configurations())
        # Leave a couple of ms to the simulator to start the action.
        self.simulator.sleep(1)
    

    def get_world_info(self):
        inst = getattr(self.simulator, self.name)
        return inst.camera.get()['visible_objects']
         #update the model of the wold (based on damages you did...)
        
      
    def close(self):
        self.simulator.quit()