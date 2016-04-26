#!/bin/bash
#sh starter.sh &
export ECG_FED=FED1
export JYTHONPATH=../framework_code/build/compling.core.jar:../framework_code/src/main/nluas/language
jython -J-Xmx4g -m analyzer ../ecg_grammars/compRobots.prefs 
#jython -m analyzer ../ecg_grammars/compRobots.prefs 
#python3 src/main/robots/robots_ui.py AgentUI 
