"""
Author: seantrott <seantrott@icsi.berkeley.edu>

"""

from robots.robot_solver import BasicRobotProblemSolver
import sys
from nluas.utils import Struct

filepath = "/Users/seantrott/icsi/nlu-core/src/main/robots/agent_templates.json"

class AgentSolver(BasicRobotProblemSolver):
    def __init__(self, args):
        BasicRobotProblemSolver.__init__(self, args)
        #self.workers = {}
        self.boss_destination = "{}_{}".format(self.federation, "ProblemSolver")
        #self.ui_address = self.boss_destination
        self.transport.subscribe(self.boss_destination, self.callback)
        self.setup_agent()
        self.read_templates(filepath)

    def setup_agent(self):
    	for name in self.world:
    		new = name.split("_instance")[0]
    		if new == self.name.lower():
    			self.agent = getattr(self.world, name)


    def callback(self, ntuple):
        self.solve(ntuple)


    def broadcast(self):
        for name in self.world:
            obj = getattr(self.world, name)
            template = self._notify
            template['protagonist'] = name
            if type(obj) != Struct:
                template['information'] = Struct(name=obj.name, pos=obj.pos)
            else:
                template['information'] = obj
            print(template)
            json_ntuple = self.decoder.convert_to_JSON(template)
            self.transport.send(self.boss_destination, json_ntuple)




if __name__ == "__main__":
    solver = AgentSolver(sys.argv[1:])