# @file arussim.launch.py
# @brief Launch file for ARUSim simulator and RViz visualization.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

# @brief Generates the launch description for the ARUSim simulation and RViz visualization.
# @return A LaunchDescription containing the configuration for nodes and arguments.
def generate_launch_description():

    # Get the package directory
    # @var package_name The name of the package where ARUSim is located.
    package_name = 'arussim'  # Replace with your package name
    
    # Define the path to the RViz configuration file
    # @var rviz_config_dir Full path to the RViz configuration file for ARUSim visualization.
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 
                                   'config', 
                                   'arussim_rviz_config.rviz')

    # Launch configuration variables
    # @var rviz_config_file A LaunchConfiguration object for RViz config file, set by a launch argument.
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=rviz_config_dir)


    # Define the path to the simulator parameters file (YAML)
    # @var config_file Full path to the parameters file used to configure ARUSim node.
    simulator_config_file = os.path.join(get_package_share_directory(package_name), 
                               'config', 
                               'simulator_params.yaml')
    
    # Define the path to the sensors parameters file (YAML)
    # @var config_file Full path to the parameters file used to configure ARUSim sensors.
    sensor_config_file = os.path.join(get_package_share_directory(package_name), 
                               'config', 
                               'sensors_params.yaml')
    
    # Declare the launch argument for overriding simulator parameters
    declare_simulator_parameters = DeclareLaunchArgument(
        'parameters',
        default_value='{}',
        description='Simulator parameters to override as a YAML formatted string'
    )

    # Function to create the arussim node with overridden parameters
    def create_arussim_node(context):
        simulator_parameters_str = LaunchConfiguration('parameters').perform(context)
        simulator_parameters = yaml.safe_load(simulator_parameters_str)
        return [
            Node(
                package='arussim',
                executable='arussim_exec',
                name='arussim',
                output='screen',
                parameters=[simulator_config_file, simulator_parameters],
                arguments=['--ros-args', '--params-file', simulator_config_file]
            )
        ]


    return LaunchDescription([
        # Declare the launch argument for the RViz config file
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=rviz_config_file,
            description='Full path to the RViz config file to use'
        ),

        # Launch the RViz node with the specified config file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        # Declare the launch argument for overriding simulator parameters
        declare_simulator_parameters,

        # Create arussim node with overridden parameters
        OpaqueFunction(function=create_arussim_node),

        # Launch the Sensors node
        Node(
            package='arussim',
            executable='sensors_exec',
            name='arussim_sensors',
            output='screen',
            parameters=[sensor_config_file],
            arguments=['--ros-args', '--params-file', sensor_config_file]
        ),

        # Launch the supervisor node
        Node(
            package='arussim',
            executable='supervisor_exec',
            name='arussim_supervisor',
            output='screen'
        ),

        # Launch the extended interface node
        Node(
            package='arussim',
            executable='extended_interface_exec',
            name='arussim_extended_interface',
            output='screen'
        )

    ])
