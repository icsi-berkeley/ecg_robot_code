"""Created on 2/19/2015 by Steve Doubleday
"""

from jpype import *
from xschema.petrinet import * 


class BaseXnet(PetriNet):
    """ all application-specific xnets should inherit from this class.
    Starts a target xnet, first registering for the standard places 
    in the base-control xnet:  Ready, Ongoing, Done
    assumes that the xnet to be run uses the standard xnet semantics, i.e., 
    is built upon base-control.xml.
    tries to set up for optional standard semantics, if present
    Finally, also calls setup method, which subclasses should override to do 
    application-specific setup prior to execution.
    """

    def __init__(self,xnet,seed,firingLimit):
        self.listener = FiringListener()
        PetriNet.__init__(self,xnet,seed,firingLimit,self.listener)
##        self.started = False
        self.readyPlace = BooleanPlace('Ready', self, 'readyStatus', self.ready)
        self.listen(self.readyPlace)
        # really should be done as part of BooleanPlace initialization
        self.readyStatus = False
        self.ongoingPlace = BooleanPlace('Ongoing', self, 'ongoingStatus', self.ongoing) 
        self.listen(self.ongoingPlace)
        self.ongoingStatus = False
        self.donePlace = BooleanPlace('Done', self, 'doneStatus', self.done) 
        self.listen(self.donePlace)
        self.doneStatus = False
        self.listenForOptionalStandardSemantics()
##        print('BaseXnet about to call setup') 
        self.setup()
        #subclasses should explicitly enable when their preconditions are met
        #self.enable() 

    def setup(self):
        pass

    def listenForOptionalStandardSemantics(self):
        #TODO try/except this, don't just assume it will work 
        self.suspendedPlace = BooleanPlace('Suspended', self, 'suspendedStatus', self.suspended)
        self.listen(self.suspendedPlace)
        self.suspendedStatus = False


    def isOngoing(self): 
        return self.ongoingStatus 

    def isSuspended(self): 
        return self.suspendedStatus 

    def isDone(self): 
        return self.doneStatus 

    def isReady(self): 
        return self.readyStatus 

    def enable(self):
        self.runner.markPlace('Enabled','Default',1)
        self.status = 'Enabled'
        
    def suspend(self):
        self.runner.markPlace('Suspend','Default',1)

    def resume(self):
        self.runner.markPlace('Resume','Default',1)

    def restart(self):
        self.runner.markPlace('Restart','Default',1)

    # These methods are evaluators, called when a place status changes
    # They should not be called by users of this class. 
    def ready(self): 
        self.status = self.readyPlace.getPlaceId()
        #print(self.status)

    def ongoing(self): 
        self.status = self.ongoingPlace.getPlaceId()
        #print(self.status, self.ongoingStatus)

    def done(self): 
        self.status = self.donePlace.getPlaceId()
        #print(self.status)

    def suspended(self): 
        self.status = self.suspendedPlace.getPlaceId()
        #print(self.status)

class MockXnet(object):
    """ mock version of BaseXnet, designed to accept the same method calls 
    as BaseXnet, and just toggle attributes as booleans. 
    initializes with the standard places, and can be passed a list of place 
    names to be initilized with status True
    """

    def buildStandardPlaces(self): 
        self.Ready = False
        self.Ongoing = False
        self.Enabled  = False
        self.Done = False
        self.Suspend = False
        self.Suspended = False
        self.Resume = False
        self.Restart = False
        self.started = False

    def __init__(self,places=None): 
        self.buildStandardPlaces()
        if places is not None: 
           for place in places: 
              self.__dict__[place] = True
        #print(self.__dict__)

    def isOngoing(self): 
        return self.Ongoing

    def isSuspended(self): 
        return self.Suspended

    def isDone(self): 
        return self.Done 

    def enable(self):
        self.Enabled = True
        
    def suspend(self):
        self.Suspend = True

    def resume(self):
        self.Resume = True

    def restart(self):
        self.Restart = True

    def start(self):
        self.enable()
        self.started = True

    def isStarted(self): 
        return self.started 

