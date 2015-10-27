export ECG_FED=FED1


#python3 src/main/robots/agent_solver.py Robot1 &
#python3 src/main/robots/agent_solver.py Robot2 &

python3 src/main/robots/morse_agent.py Robot1 &
python3 src/main/robots/morse_agent.py Robot2 &

python3 src/main/robots/boss_solver.py ProblemSolver &


python3 src/main/robots/robots_ui.py AgentUI 
#echo "PS" $PID
#sh ui_setup.sh