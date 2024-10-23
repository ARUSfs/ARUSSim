cmake_minimum_required(VERSION 3.8)
project(arussim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(arussim_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# generate executable for arussim_node.cpp
add_executable(arussim_exec src/arussim_node.cpp)
ament_target_dependencies(arussim_exec rclcpp 
                            std_msgs 
                            geometry_msgs 
                            sensor_msgs 
                            visualization_msgs 
                            arussim_msgs 
                            tf2_ros
                            tf2
                            PCL
                            pcl_conversions)

# generate executable for sensors.cpp
add_executable(sensors_exec src/sensors.cpp)
ament_target_dependencies(sensors_exec rclcpp 
                            std_msgs 
                            sensor_msgs 
                            arussim_msgs 
                            tf2_ros
                            tf2)

add_executable(supervisor_exec src/supervisor.cpp)
ament_target_dependencies(supervisor_exec rclcpp 
                            std_msgs 
                            sensor_msgs 
                            arussim_msgs 
                            tf2_ros
                            tf2)

# Install both executables
install(TARGETS
  arussim_exec
  sensors_exec
  supervisor_exec
  DESTINATION lib/${PROJECT_NAME})

# share folders location
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME})


ament_package()
