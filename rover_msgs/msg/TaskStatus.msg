# Robot -> coordinator on /team/task_status.
uint8 ACCEPTED=0     # award received, goal sent to Nav2
uint8 ACTIVE=1       # driving to the task
uint8 SUCCEEDED=2    # reached / the frontier disappeared (explored)
uint8 FAILED=3       # Nav2 aborted / unreachable -> task is penalised and re-auctioned
uint8 CANCELED=4     # preempted by a newer award
uint8 IDLE=5         # no task

std_msgs/Header header
string robot
uint32 auction_id
string task_id
uint8 status
string reason
