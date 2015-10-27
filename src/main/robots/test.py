#!/usr/bin/env python3 

import json
from morse_solver import *

f = open("test.json", "r")

data = json.load(f)

solver = MorseRobotProblemSolver(["ProblemSolver"])

for k, v in data.items():
	print(k)
	new = json.dumps(v)
	solver.solve(new)
    
