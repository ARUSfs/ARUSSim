cmake_minimum_required(VERSION 3.8)
project(arussim_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # for custom messages

# generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cmd.msg"
  "msg/Cmd4WD.msg"
  "msg/FourWheelDrive.msg"
  "msg/PointXY.msg"
  "msg/State.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES std_msgs
)



ament_package()
