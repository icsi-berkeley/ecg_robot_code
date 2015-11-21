export ECG_FED=FED1
python3 src/main/robots/cci_solver.py ProblemSolver &

export JYTHONPATH=../framework-code/build/compling.core.jar:../framework-code/src/main/nluas/language
jython -m analyzer ../ecg-grammars/compRobots.prefs &
python3 src/main/robots/robots_ui.py AgentUI 

