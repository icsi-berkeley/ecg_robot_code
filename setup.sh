# A user should set PYTHONPATH in their .bash_profile
#export PYTHONPATH=/Users/seantrott/icsi/ecg2/framework-code/src/main
export ECG_FED=FED2
#python3 src/main/robots/robot_solver.py ProblemSolver &
#python3 src/main/robots/morse_solver.py ProblemSolver
python3 src/main/robots/robot_solver_alt.py ProblemSolver
#python3 src/main/robots/morse_solver.py ProblemSolver &
#export PID=$!
#echo "PS" $PID
#sh ui_setup.sh
#python3 src/main/prog2.py AgentUI

