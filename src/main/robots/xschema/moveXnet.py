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
        self.morseChannel.updateCurrentLocation(currentX+0.0, currentY+0.0)

    def targetLocation(self, targetX, targetY):
##        print('move x, y: ', targetX, targetY)
        self.morseChannel.updateTargetLocation(targetX+0.0, targetY+0.0)

    def setSpeed(self, speed):
        self.morseChannel.setSpeed(speed+0.0)

    def setTolerance(self, tolerance):
        self.morseChannel.setTolerance(tolerance+0.0)

    def setCollide(self, collide):
        self.morseChannel.setCollide(collide)

    def moveMorse(self, command): 
        motion = command['motion']
        #where = { 'x':['x'], 'y':['y'], 'z':0.0, 'collide':False, 'speed':2.0, 'tolerance':4}
        #print(self.printts(), 'where: ', motion)
        self.robot.motion.publish(motion)
        #TODO sleeping should be done in the xnet Wait timed transition, when it works  
        #self.robot._morse.sleep(0.1)
 
    def suspendMorse(self, command):
        #print('suspendMorse')
        self.robot.motion.stop()

    def resumeMorse(self, command):
        #print('resumeMorse')
        self.robot.motion.resume()

    def restartMorse(self, command):
        #print('restartMorse')
        #TODO 
        self.robot.motion.resume()

    def updateMorsePosition(self, command):
        #print('updateMorsePosition', self.pos.x, self.pos.y)
        self.currentLocation(self.pos.x, self.pos.y)
        #self.robot._morse.sleep(0.1)
        #TODO 
        #self.robot.motion.resume()
        if self.robot.motion.get_status() == 'Arrived':
           #print('arrived!')
           self.morseChannel.setStatus(2)  
           # 2 = arrived

    def callMorse(self, commandJson): 
##        print('in callMorse') 
        command = json.loads(commandJson)
        #self.queue.put(command)
        #print('command dict: ', command)
        #TODO:  validate
        methodName = command['command']
        #print(self.printts(), ' callMorse: method to invoke: ',methodName)
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
        self.addCommand('moveMorse')
        self.addCommand('suspendMorse')
        self.addCommand('resumeMorse')
        self.addCommand('restartMorse')
        self.addCommand('updateMorsePosition')
        #print('commands: ',self.commands)

    def setup(self):
        self.morseChannel = JClass('edu.berkeley.icsi.xnet.MorseChannel')()
        proxy = JProxy('edu.berkeley.icsi.xnet.Morse', inst=self)
        self.morseChannel.setMorse(proxy)
        self.runner.setTransitionContext("Move", self.morseChannel);           
        self.runner.setTransitionContext("SuspendT", self.morseChannel);           
        self.runner.setTransitionContext("ResumeT", self.morseChannel);           
        self.runner.setTransitionContext("RestartT", self.morseChannel);           

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

