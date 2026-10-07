# 2D semantic class map (one robot's map frame), accumulated from segmentation +
# depth. Cells are row-major like nav_msgs/OccupancyGrid and share its MapMetaData
# conventions, but the grid is independent of the SLAM occupancy grid (resample by
# world coordinates when combining them).

uint8 UNKNOWN=255

std_msgs/Header header
nav_msgs/MapMetaData info

# Class index -> name (e.g. background, floor, wall, stairs).
string[] class_names

# Winning class per cell, or UNKNOWN when the cell has too few observations.
uint8[] labels

# Share of the cell's votes held by the winning class [0-100].
uint8[] confidence
