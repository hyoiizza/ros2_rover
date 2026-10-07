# One robot's price for one task.
string task_id

# Path length from the robot to the task [m]. From the Nav2 planner when
# path_planned is true, otherwise an estimate (straight line x penalty factor).
float32 path_cost
bool path_planned

# Final bid: gain - distance_weight * path_cost (- penalties). Higher wins.
float32 utility
