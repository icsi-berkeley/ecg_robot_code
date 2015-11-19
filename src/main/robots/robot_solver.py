"""

Author: seantrott <seantrott@icsi.berkeley.edu>

A RobotProblemSolver that extends the CoreProblemSolver in the NLUAS module.

Actions, like "move", should be named by predicate + action type.
Thus: query_move, command_move, etc.
Or: query_be, command_be, etc.

Questions:
(1) Keep in mind: should we route based on actions? E.g., "move to the blue box?"
This was how it was done previously, but what about queries/assertions: "he moved to the blue box",
"can you move to the blue box?", "which box did you move?"

Possible solutions:
(1) Separate "information" functions like gathering information from an n-tuple and matching to world,
from action/query functions. Thus, we'd have several functions involving "move", including:
-"do_move"
-"move_info": gather information (used in do_move)

"""

from nluas.app.core_solver import *
from nluas.utils import *
from builder import *
import sys
import random
from math import sqrt

class BasicRobotProblemSolver(CoreProblemSolver):
    def __init__(self, args):
        CoreProblemSolver.__init__(self, args)
        self.headings = dict(north=(0.0, 1.0, 0.0), south=(0.0, -1.0, 0.0), 
                    east=(1.0, 0.0, 0.0), west=(-1.0, 0.0, 0.0))
        self.world = build('mock')
        self._recent = None
        self._wh = None
        self._speed = 4
        # This depends on how size is represented in the grammar.
        self._size_cutoffs = {'big': 2,
                              'small': 1}
        self._weight_cutoffs = {'heavy': 7,
                              'light': 6}


        self._ranges = {'box': {'size': [.5, 3],
                                'weight': [1, 14]},
                         'robot': {'size': [.5, 1],
                                    'weight': [1, 4]}}

        self._home = None
        self._distance_multipliers = {'box': 1.3,
                                    'robot': .7}
        self._distance_threshold = 4
        self._attributes = ['size', 'color', 'weight']

    def euclidean_distance(self, p, q):
        """ Gets euclidean distance between objects p and q. Takes in objects themselves. """
        return sqrt(pow((p.pos.x-q.pos.x ),2) + pow((p.pos.y-q.pos.y ),2) ) 


    def set_home(self, ntuple):
        parameters = ntuple['eventDescriptor']['eventProcess']
        prot = parameters['protagonist']
        obj = self.get_described_object(prot['objectDescriptor'])
        if obj:
            self._home = obj.pos

    def solve_serial(self, parameters, predicate):
        self.route_action(parameters['process1'], predicate)
        self.route_action(parameters['process2'], predicate)

    def solve_command(self, ntuple):
        self.set_home(ntuple)
        parameters = ntuple['eventDescriptor']
        self.route_event(parameters, "command")

    def solve_query(self, ntuple):
        param = ntuple['eventDescriptor']
        self.route_event(param, "query")


    def command_move(self, parameters):
        information = self.get_move_info(parameters)
        destination = information['destination']
        if destination:
            self.move(information['protagonist'], destination['x'], destination['y'], destination['z'], 
                information['speed'], tolerance=3.5)
        else:
            print("Command_move, no destination.")

    def get_move_info(self, parameters):
        information = dict(destination=None,
                           protagonist=None,
                           speed=None)

        information['protagonist'] = self.get_described_object(parameters['protagonist']['objectDescriptor'])
        information['speed'] = parameters['speed'] * self._speed
        #spg = selparameters['spg']

        spg = parameters['spg']['spgDescriptor']

        if spg['goal']:
            information['destination'] =self.goal_info(spg['goal'], information['protagonist'])
        elif parameters['heading']:
            information['destination'] = self.heading_info(information['protagonist'], parameters['heading'], parameters['distance']['scaleDescriptor'])
        return information



    def goal_info(self, goal, protagonist=None):
        destination = dict(x=None, y=None, z=0.0)
        if "location" in goal:
            if goal['location'] == 'home':
                # Determine "home" position
                destination['x'] = self._home.x
                destination['y'] = self._home.y
            else:
                destination['x'] = goal['location'][0]
                destination['y'] = goal['location'][1]
                # Z point?
                #destination['z'] = goal['location'][0]
        elif 'objectDescriptor' in goal:
            obj = self.get_described_object(goal['objectDescriptor'], multiple=True)
            #print(obj)
            if obj:
                destination['x'] = obj.pos.x
                destination['y'] = obj.pos.y
                destination['z'] = obj.pos.z
            else:
                return None
        elif "locationDescriptor" in goal:
            properties = goal['locationDescriptor']
            position = self.get_described_position(properties, protagonist)
            destination['x'], destination['y'], destination['z'] = position[0], position[1], position[2]
        return destination

    def getpos(self, inst):
        p = getattr(getattr(self.world, inst), 'pos')
        return (p.x, p.y, p.z) 

    def heading_info(self, protagonist, heading, distance):
        n = float(distance['value'])
        # units?
        name = getattr(protagonist, 'name')
        pos = self.getpos(name)
        newpos = vector_add(pos, vector_mul(n, self.headings[heading]))
        return dict(x=newpos[0], y=newpos[1], z=newpos[2])

    def query_move(self, parameters):
        return None

    def command_push_move(self, parameters):
        #protagonist = self.get_described_object(parameters.causer['objectDescriptor'])
        info = self.get_push_info(parameters)
        if info['goal']:
            # Create self.push_to_location
            self.push_to_location(info['actedUpon'], info['goal'], info['pusher'])
            
        elif info['heading']:
            self.push_direction(info['heading'], info['actedUpon'], info['distance'], info['pusher'])

    def push_to_location(self, actedUpon, goal, pusher):
        self.identification_failure(message=self._incapable)
        og = actedUpon.pos.__dict__
        #print("Original location: {}".format(og))
        #print("Goal location: {}".format(goal))


    def push_direction(self, heading, actedUpon, distance, pusher):
        info = self.get_push_direction_info(heading, actedUpon, distance['scaleDescriptor']['value'])
        self.move(pusher, info['x1'], info['y1'], tolerance=3)
        self.move(pusher, info['x2'], info['y2'], tolerance=3, collide=True)


    def get_push_direction_info(self, heading, obj, distance):
        addpos = vector_mul(-6, self.headings[heading])
        addpos2 = vector_mul(distance, self.headings[heading])
        return {'x1': obj.pos.x + addpos[0], 
                'y1': obj.pos.y + addpos[1],
                'x2': obj.pos.x + addpos2[0],
                'y2': obj.pos.y + addpos2[1]}



    def get_push_info(self, parameters):
        heading = parameters['affectedProcess']['heading']
        pusher = self.get_described_object(parameters['protagonist']['objectDescriptor'])
        goal = parameters['affectedProcess']['spg']['spgDescriptor']['goal']
        distance = parameters['affectedProcess']['distance']
        info = dict(goal=None,
                    heading=None,
                    actedUpon=None,
                    distance=None,
                    pusher=None)
        obj = self.get_described_object(parameters['causalProcess']['actedUpon']['objectDescriptor'])
        info['actedUpon'] = obj
        if goal:
            info['goal'] = self.goal_info(goal)
        info['heading'] = parameters['affectedProcess']['heading'] #self.heading_info(obj, parameters.affectedProcess['heading'], distance)
        info['distance'] = distance
        info['pusher'] = pusher
        return info



    def distance(self, a, b):
        return sqrt(pow((a.pos.x-b.pos.x ),2) + pow((a.pos.y-b.pos.y ),2) ) 

    def get_near(self, candidates, obj):
        locations = []
        for candidate in candidates:
            if self.is_near(candidate, obj):
                locations.append(candidate)
        return locations

    def get_threshold(self, first, second):
        multiplier = (self._distance_multipliers[first.type] * first.size) + (self._distance_multipliers[second.type] * second.size)
        return self._distance_threshold + multiplier

    def is_near(self, first, second):
        """ Could be redone. Essentially an arbitrary threshold. Could be rewritten
        to evaluate "near" in a more relativistic way. 
        Could also take size into account. """
        if first == second:
            return False
        #t = self.get_threshold(first, second)
        #print(t)
        return self.distance(first, second) <= self.get_threshold(first, second)


    def get_described_position(self, description, protagonist):
        """ Returns the position/location described, e.g. "into the room", "near the box".
        (As opposed to an object described in relation to a location.) """
        obj = self.get_described_object(description['objectDescriptor'])
        if description['relation'] == 'behind':
            return self.behind(obj.pos, protagonist.pos)
        else:
            print(properties['relation'])

    def behind(self, position, reference):
        xdiff = position.x - reference.y
        ydiff = position.y - reference.y
        if abs(xdiff) > abs(ydiff):
            if xdiff>0:
                new = [position.x +3, position.y]
            elif xdiff<0:
                new = [position.x -3, position.y]
        elif abs(xdiff) < abs(ydiff):
            if ydiff>0:
                new = [position.x, position.y+3]
            elif ydiff<0:
                new = [position.x , position.y-3]
        elif abs(xdiff) == abs(ydiff):
            if ydiff>0:
                newy =  position.y+3
            elif ydiff<0:
                newy = position.y-3
            if xdiff>0:
                newx =  position.x+3
            elif xdiff<0:
                newx = position.x-3
            new = [newx, newy]
        new.append(0)
        return new




    def get_described_locations(self, candidates, description):
        obj = self.get_described_object(description['objectDescriptor'])
        if obj:
            locations = []
            if description['relation'] == 'near':
                locations = self.get_near(candidates, obj)
            elif description['relation'] == 'behind':
                # TODO: get_behind
                pass
            return locations
        else:
            return []




    def get_described_location(self, candidates, description, multiple=False):
        locations = self.get_described_locations(candidates, description)
        if multiple:
            return locations
        if len(locations) != 1:
            # TODO: what if there is more than one, what if there are none?
            return []
        else:
            return locations


    def get_described_process(self, objs, description):
        # TODO: Let's assume, for now, that we're just describing "one" parameterized action.
        description = description[0]
        described_action = description['actionary']
        for entry in self.history:
            parameters, successful = entry[0], entry[1]
            new_action = parameters['actionary']
            if successful and (new_action == described_action):
                dispatch = getattr(self, "match_{}".format(new_action))
                objs = dispatch(objs, parameters)
                return objs
        return objs

    def match_push_move(self, objs, parameters):
        """ Question: how to know whether you're discussing acted upon or protagonist?
        E.g., "move to the box Robot2 pushed", vs. "move to the robot that pushed Box2"
        TODO: Figure this out.
        """
        description = parameters['causalProcess']['actedUpon']['objectDescriptor']
        obj = self.get_described_object(description)
        if obj in objs:
            return [obj]
        return objs

    def get_described_objects(self, description, multiple=False):
        if 'referent' in description:
            if hasattr(self.world, description['referent']):
                return [getattr(self.world, description['referent'])]
        obj_type = description['type']
        objs = []
        for item in self.world.__dict__.keys():
            if hasattr(getattr(self.world, item), 'type') and getattr(getattr(self.world, item), 'type') == obj_type:
                objs += [getattr(self.world, item)]
        copy = []
        if 'color' in description:
            color = description['color']
            for obj in objs:
                if obj.color == color:
                    copy.append(obj)
            objs = copy
        kind = description['kind'] if 'kind' in description else 'unmarked'
        if 'size' in description or 'weight' in description:
            size = description['size']
            objs = self.evaluate_scalar_attribute("size", size, objs, kind)
            #objs = self.evaluate_feature(size, kind, objs)
        if 'weight' in description:
            weight = description['weight']
            objs = self.evaluate_scalar_attribute("weight", weight, objs, kind)
        if 'locationDescriptor' in description:
            objs = self.get_described_location(objs, description['locationDescriptor'], multiple=multiple)

        if "processDescriptor" in description:
            objs = self.get_described_process(objs, description['processDescriptor'])
        # TODO: Partdescriptor
        return objs



    def get_described_object(self, description, multiple=False):
        objs = self.get_described_objects(description, multiple)
        if len(objs) == 1:
            self._recent = objs[0]
            return objs[0]
        elif len(objs) > 1:
            if "givenness" in description:
                if description['givenness'] == 'typeIdentifiable' or description['givenness'] == "distinct":
                    if self._recent in objs:
                        objs.remove(self._recent)
                    return random.choice(objs)
            elif self._wh:
                message = "More than one object matches the description of {}.".format(self.assemble_string(description))
                self.identification_failure(message)
                return None
            message = "Which '{}'?".format(self.assemble_string(description))
            # TODO: Tag n-tuple
            self.request_clarification(self.ntuple, message)
            return None
        else:
            message = "Sorry, I don't know what the {} is.".format(self.assemble_string(description))
            self.identification_failure(message)
            return None

    def evaluate_scalar_attribute(self, attribute, value, objs, kind):
        if kind == "superlative":
            pass
        elif kind == "comparative":
            pass
        else:
            copy = []
            for obj in objs:
                if self.evaluate_individual_attribute(attribute, value, obj):
                    copy.append(obj)
            return copy

    def evaluate_individual_attribute(self, attribute, value, obj):
        ranges = self._ranges[obj.type][attribute]
        obj_attribute = getattr(obj, attribute)
        if value > .5:
            return obj_attribute >= (ranges[-1]-ranges[0])/2
        else:
            return obj_attribute <= (ranges[-1]-ranges[0])/2


    def get_heavyest(self, objs):
        heaviest = objs[0]
        returned = [heaviest]
        for i in objs:
            if float(i.weight) > heaviest.weight:
                heaviest = i
                returned = [heaviest]
            elif (float(i.weight) == heaviest.weight) and (i.name != heaviest.name):
                returned.append(i)
                # DO SOMETHING HERE ***
        return returned

    def get_lightest(self, objs):
        lightest = objs[0]
        returned = [lightest]
        for i in objs:
            if float(i.weight) < lightest.weight:
                lightest = i
                returned = [lightest]
            elif (float(i.weight) == lightest.weight) and (i.name != lightest.name):
                returned.append(i)
                # DO SOMETHING HERE ***
        return returned

    def get_biggest(self, objs):
        biggest = objs[0]
        returned = [biggest]
        for i in objs:
            if float(i.size) > biggest.size:
                biggest = i
                returned = [biggest]
            elif (float(i.size) == biggest.size) and (i.name != biggest.name):
                returned.append(i)
                # DO SOMETHING HERE ***
        return returned

    def get_smallest(self, objs):
        """ Returns the smallest object of input OBJS. If there are multiple smallest, it returns multiple. """
        smallest = objs[0]
        returned = [smallest]
        for i in objs:
            if float(i.size) < smallest.size:
                smallest = i
                returned = [smallest]
            elif (float(i.size) == smallest.size) and (i.name != smallest.name):
                returned.append(i)
        return returned




    def assemble_string(self, properties):
        """ Takes in properties and assembles a string, to format: "which blue box", etc. """
        ont = properties['type']
        attributes = ""
        for key, value in properties.items():   # Creates string of properties
            if key == "referent":
                return value[0].upper() + value.replace("_instance", "")[1:]
            if key == "color": # or key == "size":
                attributes += " " + value 
            if key == "location":
                attributes += " "  + value
            elif key == "locationDescriptor":
                attributes += " " + str(ont) + " " + value["relation"] + " the " + self.assemble_string(value['objectDescriptor'])
                return attributes
        return str(attributes) + " " + str(ont)

    def query_be(self, parameters):
        if "specificWh" in parameters:
            return self.eval_wh(parameters, self.ntuple['return_type'])
        else:
            msg = "Yes." if self.evaluate_condition(parameters) else "No."
            self.respond_to_query(msg)

    def query_be2(self, parameters):
        self.query_be(parameters)

    def eval_wh(self, parameters, return_type):
        num, referentType = return_type.split("::")
        protagonist = parameters['protagonist']
        predication = parameters['state']
        dispatch = getattr(self, "eval_{}".format(parameters['specificWh']))
        dispatch(protagonist, predication, num)

    def eval_where(self, protagonist, predication=None, num="singleton"):
        obj = self.get_described_object(protagonist['objectDescriptor'])
        if obj and len(obj) >= 1:
                message = "The position of the {} is: x:{}, y:{}".format(self.assemble_string(protagonist['objectDescriptor']), obj.pos.x, obj.pos.y)
                message = "The position of the {} is: ({}, {})".format(self.assemble_string(protagonist['objectDescriptor']), obj.pos.x, obj.pos.y)
                self.respond_to_query(message)

    def eval_which(self, protagonist, predication, num):
        copy = []
        objs = self.get_described_objects(protagonist['objectDescriptor'])
        negated = predication['negated']
        for obj in objs:
            if negated and not self.evaluate_obj_predication(obj, predication):
                copy.append(obj)
            elif (not negated) and self.evaluate_obj_predication(obj, predication):
                copy.append(obj)
        if len(copy) < 1:
            self.identification_failure("Failed to identify an object matching this description.")
        elif len(copy) >= 1:
            if num == "singleton" and len(copy) > 1:
                self.identification_failure(message="There is more than one item matching this description.")
            reply = ""
            index = 0
            while index < len(copy):
                reply += "{}".format(copy[index].name)
                if index < (len(copy) - 1):
                    reply += ", "
                index += 1
            self.respond_to_query(message=reply)


    def evaluate_condition(self, parameters):
        protagonist = self.get_described_object(parameters['protagonist']['objectDescriptor'])
        if protagonist:
            negated = parameters['state']['negated']
            if negated:
                return not self.evaluate_obj_predication(protagonist, parameters['state'])
            else:
                return self.evaluate_obj_predication(protagonist, parameters['state'])


    def compare_features(self, feature, comparator, obj1, obj2):
        f1 = getattr(obj1, feature)
        f2 = getattr(obj2, feature)
        return comparator(f1, f2)

    def evaluate_obj_predication(self, obj, predication):
        kind = predication['kind'] if 'kind' in predication else 'unmarked'
        for k, v in predication.items():
            if k == "size" or k == "weight":
                ranges = self._ranges[obj.type][k]
                attribute = getattr(obj, k)
                if v > .5:
                    return attribute >= (ranges[-1]-ranges[0])/2
                else:
                    return attribute <= (ranges[-1]-ranges[0])/2
            elif k == "identical":
                return self.is_identical(obj, predication['identical']['objectDescriptor'])
            elif k == 'relation':
                if v =='near':
                    if not self.is_near(obj, self.get_described_object(predication['objectDescriptor'])):
                        return False
            # TODO: "Object does not have property k". Send message?
            elif hasattr(obj, k) and getattr(obj, k) != v:
                return False
        return True


    def is_identical(self, item, objectD):
        # Checks if it's type identifiable ("is box1 a box"), then if it's elaborated ("is box1 a red box")
        # If uniquely identifiable, just matches referred objects
        # TODO: what to return if there is more than one box?
        if (objectD['givenness'] == 'typeIdentifiable'):
            if (not 'color' in objectD) and (not 'size' in objectD):
                return item.type == objectD['type']
            else:
                return item in self.get_described_objects(objectD)
        else:
            objs = self.get_described_objects(objectD)
            if len(objs) > 1:
                return item in objs
            elif len(objs) < 1:
                return False
            else:
                return item == objs[0]


    # Assertions not yet implemented for robots
    def solve_assertion(self, ntuple):
        self.decoder.pprint_ntuple(ntuple)

    def solve_conditional_imperative(self, ntuple):
        parameters = ntuple['eventDescriptor']
        condition = parameters['condition']['eventProcess']
        core = parameters['core']
        if self.evaluate_condition(condition):
            self.route_event(core, "command")

    # Conditional declaratives not yet implemented for robots
    def solve_conditional_declarative(self, ntuple):
        self.decoder.pprint_ntuple(ntuple)


    def move(self, mover, x, y, z=1.0, speed=2, tolerance=3, collide=False):
        print("{} is moving to ({}, {}, {}).".format(mover.name, x, y, z))
        mover.pos.x = x
        mover.pos.y = y
        mover.pos.z = z

if __name__ == "__main__":
    solver = BasicRobotProblemSolver(sys.argv[1:])
