echo "Generating a Problem"
rosservice call /rosplan_problem_interface/problem_generation_server

echo "Planning"
rosservice call /rosplan_planner_interface/planning_server

