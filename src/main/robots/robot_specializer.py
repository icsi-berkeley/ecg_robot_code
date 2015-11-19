"""
Author: seantrott <seantrott@icsi.berkeley.edu>
"""

from nluas.language.core_specializer import *
import os
path = os.getcwd() + "/src/main/robots/"

#filepath = "/Users/seantrott/icsi/nlu-core/src/main/robots/robot_templates.json"

class RobotSpecializer(CoreSpecializer, RobotTemplateSpecializer):
    def __init__(self, analyzer_port):
        CoreSpecializer.__init__(self, analyzer_port)
        RobotTemplateSpecializer.__init__(self)

        self.parameter_templates = {}
        self.mood_templates = {}
        self.descriptor_templates = {}
        self.event_templates = {}
        self.initialize_templates()

        #self.simple_processes['Perception'] = self.params_for_perception



    def initialize_templates(self):
        self.parameter_templates = self.read_templates(path+"parameter_templates.json")
        self.mood_templates = self.read_templates(path+"mood_templates.json")
        self.descriptor_templates = self.read_templates(path+"descriptors.json")
        self.event_templates = self.read_templates(path + "event_templates.json")

    
    """
    def params_for_perception(self, process, params):
        perceived = self.get_objectDescriptor(process.content)
        params.update(content=perceived)
        return params
    """

