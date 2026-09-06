cmake_minimum_required(VERSION 3.8)
project(swarm_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UAVState.msg"
  "msg/PoI.msg"
  "msg/LinkState.msg"
  "msg/LinkStateArray.msg"
  "msg/SwarmMessage.msg"
  "msg/FaultEvent.msg"
  "msg/RelayDecision.msg"
  "msg/Geofence.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/CollisionEvent.msg"
  "msg/GeofenceViolation.msg"
  "srv/RequestReserveSwapIn.srv"
  "srv/EstimateReachability.srv"
  "srv/GetGeofence.srv"
  "srv/GetObstacles.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
