"""
Author: seantrott <seantrott@icsi.berkeley.edu>
"""

from robots_ui import *
import json

agent = RobotUserAgent("AgentUI")

data = {}

sentences = ["Robot1, move to the blue box!",
			 "Robot2, push the blue box north!",
			 "Robot1, if Box1 is red, move to Box2!",
			 "Robot1, dash to the green box then amble to the blue one!",
			 "Robot1, if the box near the green box is red, push it south!",
			 "which box is green?",
			 "which boxes are big?",
			 "is the green box near the small red box?",
			 "Robot1, move behind the big red box!",
			 "is Box1 a box?",
			 "which box is near the green box?"]


for sentence in sentences:
	print("Parsing: '{}'".format(sentence))
	fs = agent.analyzer.parse(sentence)[0]
	print("Specializing: '{}'".format(sentence))
	ntuple = agent.specializer.specialize(fs)
	data[sentence] = ntuple



with open("manfred.json", "w") as output_file:
	json.dump(data, output_file)


agent.transport.quit_federation()
