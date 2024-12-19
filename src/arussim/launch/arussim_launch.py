import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():    

    return LaunchDescription([
        create_node(pkg='rviz2',
                    exec='rviz2',
                    config='arussim_rviz_config.rviz'),
        create_node(exec='arussim_exec',
                    config='simulator_config.yaml'),
        create_node(exec='sensors_exec',
                    config='sensors_config.yaml'),
        create_node(exec='supervisor_exec')
    ])


def create_node(pkg='arussim', exec=None, params=[], config=None): 

    if config is None:
        config = pkg + "_config.yaml"
    if exec is None:
        exec = pkg + "_exec"

    package_share_directory = get_package_share_directory('arussim')
    config_file = os.path.join(package_share_directory, "config", config)

    parameters = params if exec == 'rviz2' else [config_file] + params

    return Node(
        package=pkg,
        executable=exec,
        name=pkg,
        output="screen",
        parameters=parameters,
        arguments=['-d', config_file]
        )