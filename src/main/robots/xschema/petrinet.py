"""Created on 2/19/2015 by Steve Doubleday
"""

from jpype import *
from xschema.jvm import PetriNetJVM
import os.path
import time
import datetime

class FiringListener(object):
    """ implements PropertyChangeListener interface 
    """
    def __init__(self):
        self.javaObject = None

    def printts(self):
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S:%f')
        return st

    def printState(self):
        #print('in printState')
        stateList = []
        state = self.javaObject.getNewValue().state
        it = state.getPlaces().iterator()
        while it.hasNext(): 
           place = it.next()
           tokens = state.getTokens(place) 
           keys = tokens.keySet()
           itk = keys.iterator()
           while itk.hasNext(): 
              key = itk.next()
              stateList.append((place,key,tokens.get(key).intValue()))
        #print('updated state: ', end='')
        #print(self.printts(), stateList)
    
    def propertyChange(self,javaObject):
        #print('firing listener property change: ', javaObject.getPropertyName()) 
        if javaObject.getPropertyName() == 'state updated':  
            self.javaObject = javaObject
            self.printState()


class PlaceListener(object):
    """ implements PropertyChangeListener interface 
    Currently assumes only one token color    
    """
    def __init__(self, place, petrinet, targetField, evaluator, evaluatorArgs=None):
        self.javaObject = None
        self.placeId = place
        self.petrinet = petrinet
        self.runner = petrinet.runner
        self.targetField = targetField
        self.evaluator = evaluator
        self.evaluatorArgs = evaluatorArgs
    
    def call_evaluator(self):
        return True

    def printts(self):
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S:%f')
        return st

    def convertObjectToDigit(self):
        # Fixme: ignores the token(s), just returns a single count
        tokens = self.javaObject.getNewValue()
        keys = tokens.keySet()
        it = keys.iterator()
        # while it.hasNext(): 
        if it.hasNext():
           key = it.next()
           return tokens.get(key).intValue()
        else: 
           return 0 

    def updateAttribute(self, value): 
        self.petrinet.__dict__[self.targetField] = value 
        #print('attribute  updated, __dict__: ',end='')
        #print(self.petrinet.__dict__)

    def propertyChange(self,javaObject):
        #print('petrinet.propertyChange: ',javaObject.getPropertyName())
        #print(self.printts(), 'updated place ', end='')
        #print(self.placeId, end='')
        #print(': ', end='')
        #print(javaObject.getNewValue())
        if javaObject.getPropertyName() == 'tokens':
            self.javaObject = javaObject
            self.count = self.convertObjectToDigit()
            # updateTargetField delegated to subclasses
            self.updateTargetField()
            if self.call_evaluator():
                if self.evaluatorArgs is None:
                    self.evaluator()
                else: 
                    self.evaluator(self.evaluatorArgs)

    def getPlaceId(self):
        return self.placeId
 
    def toString(self): 
        return 'place listener for '+self.placeId+' in petrinet '+str(self.petrinet)

class BooleanPlace(PlaceListener): 
    """ interprets a place as a boolean value
    """
                 
    def updateTargetField(self):
        if self.count == 0: 
            self.value = False
        else: 
            self.value = True
        self.updateAttribute(self.value)

class BooleanPlaceTrue(BooleanPlace):
    """ Boolean place only created to have an evaluator called,
        and only when true 
    """
    def __init__(self, place, petrinet, evaluator, evaluatorArgs=None):
        BooleanPlace.__init__(self, place, petrinet, None, evaluator, evaluatorArgs=None)        

    def call_evaluator(self):
        return self.value

    def updateAttribute(self, value):
        # override update to the petrinet
        # assumes some other PlaceListener is doing the update
        pass
        
class NumberPlace(PlaceListener): 
    """ inteprets a place marking as an integer
    """

    def updateTargetField(self):
        self.updateAttribute(self.count)

class PetriNet(object):
    """ interface to a Petri net; JVM will be started if needed
    """ 
    def __init__(self,petriNetPath,seed,firingLimit,listener):
        jvm = PetriNetJVM() 
        self.runner = JClass('uk.ac.imperial.pipe.runner.PetriNetRunner')(petriNetPath)
        self.runner.setSeed(seed)
        self.runner.setFiringLimit(firingLimit)
        proxy = JProxy('java.beans.PropertyChangeListener', inst=listener)
        self.runner.addPropertyChangeListener(proxy)
        self.started = False
        
    def printts(self):
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S:%f')
        return st

    def start(self):
        self.started = True
        self.runner.run()

    def isStarted(self):
        return self.started

    def listen(self, placeListener):
        proxy = JProxy('java.beans.PropertyChangeListener', inst=placeListener)
        self.runner.listenForTokenChanges(proxy, placeListener.getPlaceId()) 
        #print('now listening for token changes for ',end='')
        #print(placeListener.getPlaceId())

    def listenAllStateChanges(self, firingListener):
        proxy = JProxy('java.beans.PropertyChangeListener', inst=firingListener)
        self.runner.addPropertyChangeListener(proxy)
