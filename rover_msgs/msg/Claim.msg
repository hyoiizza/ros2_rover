# Robot <-> robot on /team/claim, used in standalone mode (no coordinator):
# "I am going here" so the other robot avoids that area.
std_msgs/Header header    # frame_id: world
string robot
TaskCandidate task
bool release              # true: the claim is dropped (done / failed / canceled)
