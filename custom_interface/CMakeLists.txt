cmake_minimum_required(VERSION 3.5)
project(custom_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DiceResult.msg"
  "srv/PlaceBet.srv"
  "action/MoveArm.action"
)

ament_package()
