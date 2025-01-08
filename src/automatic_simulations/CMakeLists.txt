cmake_minimum_required(VERSION 3.8)
project(automatic_simulations)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# generate executable for arussim_node.cpp
add_executable(automatic_simulations_exec src/automatic_simulations.cpp)
ament_target_dependencies(automatic_simulations_exec
  rclcpp
  std_msgs
)

# Install both executables
install(TARGETS
  automatic_simulations_exec
  DESTINATION lib/${PROJECT_NAME})

# share folders location
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})


ament_package()
