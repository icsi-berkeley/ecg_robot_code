"""
Author: seantrott <seantrott@icsi.berkeley.edu>
"""

from nluas.language.core_specializer import *
import os
dir_name = os.path.dirname(os.path.realpath(__file__))

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
        self.parameter_templates = self.read_templates(os.path.join(dir_name, "parameter_templates.json"))
        self.mood_templates = self.read_templates(os.path.join(dir_name, "mood_templates.json"))
        self.descriptor_templates = self.read_templates(os.path.join(dir_name, "descriptors.json"))
        self.event_templates = self.read_templates(os.path.join(dir_name, "event_templates.json"))


