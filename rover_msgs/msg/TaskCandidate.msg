# One exploration task a robot could take: a frontier point today, a space
# (room / corridor) later. Positions are in the frame of the enclosing header
# (the shared "world" frame).

# Unique id. Robots use "<robot>/<seq>"; the coordinator re-ids merged candidates as "t<seq>".
string id

# Robot that found it ("" for candidates created by the coordinator).
string source_robot

# Goal position for Nav2 (world frame).
geometry_msgs/Point position

# Expected information gain (frontier size / exploration score). Higher is better.
float32 gain

# Space-level exploration (later): id of the space this task belongs to, and its semantic label.
string region_id
string label

# Joint goal selection (rover_multi/goal_selection.py):
# unknown cells within a fixed radius of the goal in the reporting robot's map (raw gain G).
float32 unknown_cells
# Search-task relevance of the frontier's observation [0-1] (VLM), valid when semantic_observed.
float32 semantic_relevance
bool semantic_observed
