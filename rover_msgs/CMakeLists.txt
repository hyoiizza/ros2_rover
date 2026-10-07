cmake_minimum_required(VERSION 3.5)
project(rover_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorsCommand.msg"
  # Multi-robot task allocation (see rover_multi)
  "msg/TaskCandidate.msg"
  "msg/TaskCandidates.msg"
  "msg/AuctionAnnouncement.msg"
  "msg/BidItem.msg"
  "msg/Bid.msg"
  "msg/Assignment.msg"
  "msg/Award.msg"
  "msg/TaskStatus.msg"
  "msg/Heartbeat.msg"
  "msg/Claim.msg"
  "msg/LinkQuality.msg"
  # VLM semantic relevance (see rover_vlm)
  "msg/FrontierRelevance.msg"
  # Semantic mapping (see rover_semantic)
  "msg/SemanticGrid.msg"
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

ament_package()
