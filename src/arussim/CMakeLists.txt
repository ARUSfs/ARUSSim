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
find_package(rviz_common REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(common_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MAIN_MOC_FILES
  include/arussim/main_interface.hpp
)
qt5_wrap_cpp(PLOT_MOC_FILES
  include/arussim/plot_interface.hpp
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(main_interface_exec
  src/main_interface.cpp
  ${MAIN_MOC_FILES}
)
target_include_directories(main_interface_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
add_library(plot_interface_exec
  src/plot_interface.cpp
  ${PLOT_MOC_FILES}
)
target_include_directories(plot_interface_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(main_interface_exec
  pluginlib
  rviz_common
  arussim_msgs
)
ament_target_dependencies(plot_interface_exec
  pluginlib
  rviz_common
  arussim_msgs
  common_msgs
)
install(TARGETS main_interface_exec
        EXPORT export_main_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(TARGETS plot_interface_exec
        EXPORT export_main_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES main_interface.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES plot_interface.xml
        DESTINATION share/${PROJECT_NAME}
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


ament_export_include_directories(include)
ament_export_targets(export_main_interface)
pluginlib_export_plugin_description_file(rviz_common main_interface.xml)
pluginlib_export_plugin_description_file(rviz_common plot_interface.xml)

ament_package()
