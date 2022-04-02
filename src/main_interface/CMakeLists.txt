cmake_minimum_required(VERSION 3.5)
project(main_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallDistance.msg"
  "msg/Coordination.msg"
  "msg/GameController.msg"
  "msg/Odometry.msg"
  "msg/RobotCaseState.msg"
  "msg/RobotGridPos.msg"
  "msg/RobotStatus.msg"
  "msg/RobotPos.msg"
  "msg/ServPos.msg"
  "msg/WalkCommand.msg"
  "msg/HeadCommand.msg"
  "msg/MotionCommand.msg"
  "msg/Accelero.msg"
  "msg/Gyroscope.msg"
  "msg/Button.msg"
  "msg/Angle.msg"
  "msg/BallCoor.msg"
  "msg/Localization.msg"
  "msg/VisionBall.msg"
  "msg/VisionGoal.msg"
  "msg/VisionLcross.msg"
  "msg/VisionXcross.msg"
  "msg/VisionTcross.msg"
  "msg/HeadingCoor.msg"
  "msg/RobotCoor.msg"
  "msg/DeltaPos.msg"
)

ament_package()