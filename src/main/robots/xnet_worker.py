from xschema.moveXnet import MoveXnet
from xschema.petrinet import BooleanPlaceTrue

from threading import *
import jpype

class XnetWorker(object):
    """ Basic XnetWorker, listens for changes and communicates with Solver.  """

    def __init__(self):
        self.command_processor_listener = None
        self.mock = False
        self.solver_xnet_done = None
        self.solver_xnet_suspended = None
        self.started = False
        self.xnet = None


    def build_boolean_place_true(self, place, solver_xnet_method):
        if not solver_xnet_method == None:
            boolean_place = BooleanPlaceTrue(place, self.xnet, solver_xnet_method)
            self.xnet.listen(boolean_place)
            return boolean_place
        else:
            return None

    def build_boolean_places(self):
        self.done_place = self.build_boolean_place_true('Done', self.solver_xnet_done)
        self.suspended_place = self.build_boolean_place_true('Suspended', self.solver_xnet_suspended)        


    def build_xnet(self, xnet):
        self.xnet = xnet
        self.jvmAttached = False
        self.attach_jvm()
        self.build_boolean_places()
##        print('moveXnet built') 
        if not self.command_processor_listener == None: 
            self.xnet.listenAllStateChanges(self.command_processor_listener)


    def add_command_processor_as_listener(self, listener):
        # listen to all state changes; used to get control to check for new commands
##        print('in simulator.addCommandProcessorAsListener')
        self.command_processor_listener = listener
        if not self.xnet == None:
            self.xnet.listenAllStateChanges(self.command_processor_listener)

    def add_solver_as_evaluator(self, xnet_done, xnet_suspended):
        self.solver_xnet_done = xnet_done
        self.solver_xnet_suspended = xnet_suspended
        if not self.xnet == None:
            self.build_boolean_places()

    def set_mock_xnet(self, mockXnet): 
        self.mock = True
        self.mockXnet = mockXnet

    def get_xnet(self): 
        if self.mock: 
           return self.mockXnet
        else: 
           return self.xnet

    def attach_jvm(self):
        # attachThreadToJVM() can't be called at __init__; JRE dump
        if not self.jvmAttached:
            jpype.attachThreadToJVM()
            self.jvmAttached = True
##            print('JVM attached to thread') 

    def stop(self):
        self.attach_jvm()
        if self.get_xnet().isOngoing():
           #print("stopping")
           self.get_xnet().suspend()
        else:
           pass 

    def resume(self):
        self.attach_jvm()
        if self.get_xnet().isSuspended():
           self.get_xnet().resume()
        else: 
           pass

    def isOngoing(self): 
        return self.get_xnet().isOngoing()

    def isSuspended(self): 
        return self.get_xnet().isSuspended()

    def isDone(self): 
        return self.get_xnet().isDone() 

    def enable(self):
        self.attach_jvm()
        self.get_xnet().enable()
        
    def suspend(self):
        self.attach_jvm()
        self.get_xnet().suspend()

    def resume(self):
        self.get_xnet().resume()

    def restart(self):
        self.get_xnet().restart()

    def start(self):
        self.enable()
        self.get_xnet().start()
        self.started = True

    def isStarted(self): 
        return self.started 