#!/bin/bash

export ECG_FED=FED1
python3 src/main/robots/robot_solver.py ProblemSolver &
python3 src/main/robots/robots_ui.py AgentUI &
python3 src/main/robots/robots_text.py TextAgent

