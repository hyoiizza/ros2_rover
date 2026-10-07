# Every node -> everyone on /team/heartbeat (best effort, periodic).
uint8 ROLE_MEMBER=0          # lower layer only
uint8 ROLE_COORDINATOR=1     # also runs the upper layer (leader robot)

uint8 MODE_COORDINATED=0     # following the coordinator's awards
uint8 MODE_STANDALONE=1      # coordinator lost: exploring alone, deconflicting via /team/claim

std_msgs/Header header       # frame_id: world
string robot
uint8 role
uint8 mode
geometry_msgs/Pose pose      # robot pose in world
string current_task_id
