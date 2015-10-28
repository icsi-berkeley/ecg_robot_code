#sh starter.sh &
export ECG_FED=FED2
export JYTHONPATH=../framework-code/build/compling.core.jar:../framework-code/src/main/nluas/language
jython -m analyzer ../../ecg-grammars/compRobots.prefs #&
export PID=$!
#echo "Analyzer" $PID
#python3 src/main/robots/robots_ui.py AgentUI 
