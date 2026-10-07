# Coordinator -> robots on /team/auction/award (reliable + transient_local,
# so a robot that (re)joins gets the latest assignments).
# Always carries the full current assignment set, one entry per robot.
std_msgs/Header header    # frame_id: world
uint32 auction_id
Assignment[] assignments
