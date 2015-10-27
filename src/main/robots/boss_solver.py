"""
Author: seantrott <seantrott@icsi.berkeley.edu>

"""

from robots.robot_solver import BasicRobotProblemSolver
import sys
import json

class BossSolver(BasicRobotProblemSolver):
    def __init__(self, args):
        BasicRobotProblemSolver.__init__(self, args)
        self.workers = {}
        self.setup_workers()

    def setup_workers(self):
        self.workers['robot1_instance'] = "{}_Robot1".format(self.federation)
        self.workers['robot2_instance'] = "{}_Robot2".format(self.federation)
        for key, value in self.workers.items():
            self.transport.subscribe(value, self.feedback)

    def feedback(self, json_ntuple):
        """ Receiving feedback from workers. """
        ntuple = self.decoder.convert_JSON_to_ntuple(json_ntuple)
        if ntuple['kind'] == "notification":
            info = ntuple['information']
            self.world.__dict__[ntuple["protagonist"]] = info
            #print("\n --- \n")
            print(info)

    def callback(self, json_ntuple):
        ntuple = self.decoder.convert_JSON_to_ntuple(json_ntuple)
        predicate_type = ntuple['predicate_type']
        if predicate_type in ['query', 'assertion']:
            self.solve(json_ntuple)
        else:
            self.route(ntuple)

    def modify_ntuple(self, params, agent):
        new_agent = getattr(self.world, agent)
        if params['action'] == "push_move":
            params['causer']['objectDescriptor']['referent'] = agent
            params['causer']['objectDescriptor']['type'] = new_agent.type
            params['protagonist'] = params['causer']

        elif params['action'] == "move":
            params['protagonist']['objectDescriptor']['referent'] = agent
            params['protagonist']['objectDescriptor']['type'] = new_agent.type
        return params        

    def route_parameters(self, new, parameters):
        for params in parameters:
            if params['collaborative']:
                pass
                # TODO: Handle collaborative process
            else: 
                agents = self.determine_agent(params)
                for agent in agents:
                    new_param = self.modify_ntuple(params, agent)
                    new['parameters'] = [new_param]
                    json_ntuple = self.decoder.convert_to_JSON(new)
                    destination = self.workers[agent]
                    self.transport.send(destination, json_ntuple)



    def determine_agent(self, param, heuristic=None):
        evaluator = 0
        best_agents = []
        if not heuristic:
            heuristic = self.euclidean_distance 
        if param['action'] == "push_move":
            properties = param['causalProcess']['acted_upon']['objectDescriptor']
        elif param['action'] == "move":
            if param['goal'] != None:
                properties=param['goal']['objectDescriptor']
            elif param['heading']:
                return list(self.solvers.keys())
        obj = self.get_described_object(properties, multiple=True)
        for name in self.workers.keys():
            agent = getattr(self.world, name)
            if obj:
                new_evaluator = heuristic(obj, agent)
                if (evaluator == 0) or (evaluator > new_evaluator):
                    evaluator = new_evaluator
                    #best_agents.append(agent)
                    best_agent = name
        return [best_agent]




    def build_new_ntuple(self, old_ntuple, agent):
        new_ntuple = dict(return_type=old_ntuple["return_type"])
        if old_ntuple['predicate_type'] == "conditional_imperative":
            if self.evaluate_condition(old_ntuple['parameters'][0]['condition'][0]):
                new_ntuple['predicate_type'] = "command"
                self.route_parameters(new_ntuple, old_ntuple['parameters'][0]['command'])
        else:
            new_ntuple['predicate_type'] = "command"
            self.route_parameters(new_ntuple, old_ntuple['parameters'])


    def route(self, ntuple):
        agent_desc = self.identify_agent(ntuple)
        if 'referent' in agent_desc['objectDescriptor']:
            agent = agent_desc['objectDescriptor']['referent']
            if agent in self.workers:
                destination = self.workers[agent]
                self.transport.send(destination, self.decoder.convert_to_JSON(ntuple))
            elif agent in ["team_instance", "unknown", "joint"]:
                self.build_new_ntuple(ntuple, agent)



    def identify_agent(self, ntuple):
        if ntuple['parameters'][0]['kind'] in ['conditional_imperative', "conditional_declarative"]:
            return ntuple['parameters'][0]['command'][0]['protagonist']
        return ntuple['parameters'][0]['protagonist']       
        


if __name__ == "__main__":
    boss = BossSolver(sys.argv[1:])