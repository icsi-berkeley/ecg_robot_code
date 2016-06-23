"""
A Solver, and Solver Command Processor, that incorporates Xnets.

(In development)

"""

from queue import * 
from threading import *
import time

from nluas.app.core_solver import CoreProblemSolver
import sys
from robots.xnet_worker import *

class SolverCommandProcessor(Thread):
    """ Long-running thread to run solver commands that use xnets 
        Reads queue of commands built by the XnetMorseSolver
        Assumes there is only one of these threads running per command Q
        Executes the commands against the solver 
        Command is a tuple:  (method, **kwargs) 
        Ask the thread to stop by calling its join() method.
        Gets control to check for more user commands
         whenever the simulator's current xnet has a property change
        queue entry format:  (priority, (method,parameters))
        
        commandQ:  PriorityQueue created by solver
        simulator:  BaseSimulator or descendent
        solver:  DispatchingProblemSolver
    """   
    def __init__(self, commandQ, solver, simulator):
        super(SolverCommandProcessor, self).__init__()
        self.SLEEP_TIME = 0.1
        self.commandQ = commandQ
        self.stoprequest = Event()
        self.inst = simulator
        self.solver = solver
        self.inst.add_command_processor_as_listener(self)
        self.inst.add_solver_as_evaluator(self.xnet_done, self.xnet_suspended)
        self.initialize_flags()
        self.command_done = True
        self.last_command = (solver.get_priority(),1,(None))
##        print('SolverCommandProcessor init complete')

    def checkAndRunCommand(self):
        if not self.commandQ.empty():
            if (self.next_command_priority() < self.last_command_priority()) or self.command_done or self.other_command_done or (self.continue_dequeue and self.suspended_command_completed):
                command = self.commandQ.get()
                self.last_command = command
                method = command[2][0]
                args = command[2][1]
                if self.command_done: 
                   self.command_done = False
                if self.other_command_done:
                   self.other_command_done = False
                if not args == None:
                    return method(args) # Testing, for queries, ST (6/17/16)
                else:
                    return method()
        else:
            self.initialize_flags()

# if we suspended a command, then we wait for it.
# Once it completes, then we can proceed either with xnet or other command finishing

    def initialize_flags(self):
        self.set_suspended_expected(False)
        self.set_continue_expected(False)
        self.continue_dequeue = False
        self.suspended_command_completed = False
        self.other_command_done = False


    def xnet_done(self):
        self.command_done = True
        if self.continue_dequeue:
            self.suspended_command_completed = True
            self.checkAndRunCommand()
            self.continue_dequeue = False
            self.suspended_command_completed = False
        else: 
            self.checkAndRunCommand()
##        print('done!')

    def set_other_command_done(self):
        self.other_command_done = True

    def xnet_suspended(self):
        if self.is_suspended_expected():
            self.set_suspended_expected(False)
            self.set_continue_expected(True)
            self.continue_dequeue = False
        else:
            self.new_solve_required = True
##        print('suspended!') 

    def next_command_priority(self):
        return self.commandQ.queue[0][0]

    def last_command_priority(self):
        return self.last_command[0]

    def is_command_done(self):
        return self.command_done

    def get_last_command(self):
        return self.last_command

    def is_suspended_expected(self):
        return self.suspended_expected

    def set_suspended_expected(self, expected):
        self.suspended_expected = expected

    def is_continue_expected(self):
        return self.continue_expected

    def set_continue_expected(self, expected):
        self.continue_expected = expected
        if not self.continue_expected:
            self.continue_dequeue = True

            
    def propertyChange(self,javaObject):
        if javaObject.getPropertyName() == 'state updated':  
            #print('SolverCommandProcessor propertyChange, about to check and run command')
            self.checkAndRunCommand()

    def run(self):
        while not self.stoprequest.isSet():
##            print('SolverCommandProcessor in run')
            return_value = self.checkAndRunCommand()
            if return_value:
                self.solver.respond_to_query(return_value)
            time.sleep(self.SLEEP_TIME)
##            try: 
##                command = self.commandQ.get(True, 0.1) 
##                method = command[0]
##                print('method: ', method)
##                args = command[1]
##                print('args: ', args)
##                if not args == None:
##                    method(**args)
##                else:
##                    method()
##            except Empty:
##                continue
               
    def join(self, timeout=None):
        self.stoprequest.set()
        super(SolverCommandProcessor, self).join(timeout)


class QueueSolver(CoreProblemSolver):
    """Puts problems as ntuples on a queue for dispatching by a SolverCommandProcessor 

    Inherits from CoreProblemSolver. The main difference is that route_dispatch now puts the dispatch function
    and parameters on a command Queue, instead of solving it immediately.
    """
    def __init__(self, args):
        self.DEFAULT_PRIORITY = 100
        #self.SLEEP_TIME = 0.1
        debug = 0
        self.commandQ = PriorityQueue()
        #CoreProblemSolver.__init__(self, args)
##        print('DispatchingProblemSolver created PriorityQueue: ', self.commandQ)
        self.reset_priority()
        self.xnet_worker = XnetWorker()
        self.tiebreaker = 0
##        self.build_solver_processor(self.commandQ, self, None)

    def build_solver_processor(self, queue, solver, simulator):
        self.solver_command_processor = SolverCommandProcessor(queue, solver, simulator)
        self.solver_command_processor.start()


    def get_priority(self):
        return self.priority

    def reset_priority(self):
        self.priority = self.DEFAULT_PRIORITY

    def get_tiebreaker(self):
        self.tiebreaker += 1
        return self.tiebreaker

    def bump_priority(self):
        self.priority -= 1
        if self.priority < 1:
            self.priority = 1

    def command_move(self, parameters):
        print(parameters)

    def route_dispatch(self, dispatch_function, parameters):
        """ Puts dispatch function and parameters on commandQ. """
        if (not self.commandQ.empty()) or (not self.solver_command_processor.is_command_done()):
            self.bump_priority()
        self.commandQ.put((self.priority, self.get_tiebreaker(), (dispatch_function, parameters)))


#if __name__ == "__main__":
#	solver = QueueSolver(sys.argv[1:])




