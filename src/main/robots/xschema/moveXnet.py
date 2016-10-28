"""Created on 4/9/2015 by Steve Doubleday
"""

from jpype import *
import json
from xschema.baseXnet import * 
import sys
from nluas.utils import update, Struct
from queue import Queue
import time

class MoveXnet(BaseXnet):
    """ move xnet 
    """ 
    def __init__(self, robot):
##        print('MoveXnet init')
        BaseXnet.__init__(self,'xnet/move.xml',123456,10000)
        self.robot = robot
        self.robot.robot_pose.subscribe(self.setpos) # what is this doing?
        #self.queue = Queue()
        #print('MoveXnet robot name: ', self.robot.name) 

    def setpos(self, pos):
        self.pos = Struct(**pos)

    def currentLocation(self, currentX, currentY):
        self.motionChannel.updateCurrentLocation(currentX+0.0, currentY+0.0)

    def targetLocation(self, targetX, targetY):
##        print('move x, y: ', targetX, targetY)
        self.motionChannel.updateTargetLocation(targetX+0.0, targetY+0.0)

    def setSpeed(self, speed):
        self.motionChannel.setSpeed(speed+0.0)

    def setTolerance(self, tolerance):
        self.motionChannel.setTolerance(tolerance+0.0)

    def setCollide(self, collide):
        self.motionChannel.setCollide(collide)

    def moveMover(self, command): 
        motion = command['motion']
        #where = { 'x':['x'], 'y':['y'], 'z':0.0, 'collide':False, 'speed':2.0, 'tolerance':4}
        #print(self.printts(), 'where: ', motion)
        self.robot.motion.publish(motion)
        #TODO sleeping should be done in the xnet Wait timed transition, when it works  
        #self.robot._morse.sleep(0.1)
 
    def suspendMover(self, command):
        #print('suspendMover')
        self.robot.motion.stop()

    def resumeMover(self, command):
        #print('resumeMover')
        self.robot.motion.resume()

    def restartMover(self, command):
        #print('restartMover')
        #TODO 
        self.robot.motion.resume()

    def updateMoverPosition(self, command):
        #print('updateMoverPosition', self.pos.x, self.pos.y)
        self.currentLocation(self.pos.x, self.pos.y)
        #self.robot._morse.sleep(0.1)
        #TODO 
        #self.robot.motion.resume()
        if self.robot.motion.get_status() == 'Arrived':
           #print('arrived!')
           self.motionChannel.setStatus(2)  
           # 2 = arrived

    def callMover(self, commandJson): 
##        print('in callMover') 
        command = json.loads(commandJson)
        #self.queue.put(command)
        #print('command dict: ', command)
        #TODO:  validate
        methodName = command['command']
        #print(self.printts(), ' callMover: method to invoke: ',methodName)
        #print(self.__dict__) 
        self.commands[methodName](self,command)
        # yield to other threads 
        time.sleep(0) 
        # caller can request a callback by overridding ongoing(), etc. 

    def addCommand(self, command):
        #self.commands[command] = self.__dict__[command]
        #print('module: ', sys.modules[__name__])
        #print('module dict : ', sys.modules[__name__].__dict__)
        #print('module dict MoveXnet: ', sys.modules[__name__].MoveXnet.__dict__)
        #self.commands[command] = getattr(sys.modules[__name__], command)
        # sure would be nice if there were a simpler way to do this....
        self.commands[command] = sys.modules[__name__].MoveXnet.__dict__[command]

    def buildCommands(self):
        #print(self.__dict__) 
        self.commands = {}
        self.addCommand('moveMover')
        self.addCommand('suspendMover')
        self.addCommand('resumeMover')
        self.addCommand('restartMover')
        self.addCommand('updateMoverPosition')
        #print('commands: ',self.commands)

    def setup(self):
        self.motionChannel = JClass('edu.berkeley.icsi.xnet.MotionChannel')()
        proxy = JProxy('edu.berkeley.icsi.xnet.Mover', inst=self)
        self.motionChannel.setMover(proxy)
        self.runner.setTransitionContext("Move", self.motionChannel);           
        self.runner.setTransitionContext("SuspendT", self.motionChannel);           
        self.runner.setTransitionContext("ResumeT", self.motionChannel);           
        self.runner.setTransitionContext("RestartT", self.motionChannel);           

        self.buildCommands()
##        print('MoveXnet setup complete')

    def start(self):
        # moved from baseXnet, so done explicitly
##        print('movexnet started')
        self.enable()  
        PetriNet.start(self)

class MockMoveXnet(MockXnet):
    """ move xnet 
    """ 
    def targetLocation(self, x, y):
        self.__dict__['x'] = x
        self.__dict__['y'] = y
        
    def setSpeed(self, speed): 
        self.__dict__['speed'] = speed

    def setTolerance(self, tolerance):
        self.__dict__['tolerance'] = tolerance

    def setCollide(self, collide): 
        self.__dict__['collide'] = collide

