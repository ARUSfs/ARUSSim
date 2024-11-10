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
find_package(nlohmann_json REQUIRED)
find_package(Qt5Widgets REQUIRED)
qt5_wrap_cpp(MOC_SOURCES include/arussim/extended_interface.hpp)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# generate executable for arussim_node.cpp
add_executable(arussim_exec src/arussim_node.cpp src/vehicle_dynamics.cpp)
ament_target_dependencies(arussim_exec rclcpp 
                            std_msgs 
                            geometry_msgs 
                            sensor_msgs 
                            visualization_msgs 
                            arussim_msgs 
                            tf2_ros
                            tf2
                            PCL
                            pcl_conversions
                            nlohmann_json)

# generate executable for sensors.cpp
add_executable(sensors_exec src/sensors.cpp)
ament_target_dependencies(sensors_exec rclcpp 
                            std_msgs 
                            sensor_msgs 
                            arussim_msgs 
                            tf2_ros
                            tf2)

# generate executable for supervisor.cpp
add_executable(supervisor_exec src/supervisor.cpp)
ament_target_dependencies(supervisor_exec rclcpp 
                            std_msgs 
                            sensor_msgs 
                            arussim_msgs 
                            tf2_ros
                            tf2)

# generate executable for extended_interface.cpp
add_executable(extended_interface_exec src/extended_interface.cpp ${MOC_SOURCES})
ament_target_dependencies(extended_interface_exec rclcpp 
                            Qt5Widgets
                            std_msgs)

# Install both executables
install(TARGETS
  arussim_exec
  sensors_exec
  supervisor_exec
  extended_interface_exec
  DESTINATION lib/${PROJECT_NAME})

# share folders location
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME})


ament_package()
