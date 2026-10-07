# Coordinator -> robots on /team/auction/announce: "bid on these tasks".
std_msgs/Header header    # frame_id: world
uint32 auction_id

# Robots expected to bid. A robot not listed ignores the announcement.
string[] robots

# Merged, de-duplicated tasks that are open for bidding.
TaskCandidate[] tasks

# Bids arriving after this time are ignored.
builtin_interfaces/Time deadline
