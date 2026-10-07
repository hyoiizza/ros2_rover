# Search-task relevance of one robot's frontiers (rover_vlm), aligned with the
# goals of the explore/frontier_candidates message it answers.

std_msgs/Header header        # frame_id: the robot's map frame

geometry_msgs/Point[] positions
# S_j in [0, 1]: normalized VLM image-text relevance of the observation of the frontier.
float32[] relevance
# false: no keyframe shows the frontier yet (relevance is meaningless).
bool[] observed
