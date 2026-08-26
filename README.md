from here i kept the same regressable format, but this time we will have contineous time actions in contineous space, the action move to will be modeled as a single action from start to goal,
the motion will be modelled as a nominal trajectory affected by gaussian noise. we will evaluate the same parameters as before, but this time over the contineous trajectory. additionally, by
passing to contineous space the map will be obtained from a nav_msgs/OccupancyGrid, that will be used and tranformed in a series of obstacle polygones representing the obstacles for the basic
action theory to work. 

command to run all

python3 occgrid_to_problog.py my_map.yaml --out obstacles_generated.pl --inline moveto_continuous.pl --start X Y --goal X Y
# hand-edit control_points/1 in moveto_continuous.pl
python3 run_plan_continuous_safety.py moveto_continuous.pl

Introduced battery consumption as a linear decreasing model affected by stochastic error where the buttery consumption rate is dependent by the action being executed.

Modified the action moveto to work as a template action, where the different reasons/conditions for which the action may fail can be provided as an argument, still they need to be formalized inside the file: for example a collision with an obstacle must be axiomatized based on the robot position and the map, so that we can simply pass to the action the condition collision.

added action outcome (status) to reflect success/failure in BT, also created the sequence  and fallback process.# BT_project
