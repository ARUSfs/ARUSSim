import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit


def generate_launch_description():    
    return LaunchDescription([
        create_node(pkg='rviz2',
                    exec='rviz2',
                    config='arussim_rviz_config.rviz',
                    config_pkg='arussim'),
        create_node(exec='arussim',
                    name='simulator',
                    config='simulator_config.yaml'),
        create_node(exec='sensors',
                    config='sensors_config.yaml'),
        create_node(exec='supervisor')
    ])


def create_node(pkg='arussim', exec=None, params=[], name=None, config_pkg=None, config=None, condition=None): 
    if config_pkg is None:
        config_pkg = pkg
    package_share_directory = get_package_share_directory(config_pkg)
    if name != 'arussim':
        name = exec
    if exec != 'rviz2':
        exec = exec + "_exec"
    if config is not None:
        config_file = os.path.join(package_share_directory, "config", config)
    else:
        config_file = None
    if exec == 'rviz2':
        parameters = params
        arguments = ['-d', config_file]
    else:
        parameters = [config_file] + params if config_file else params
        arguments = []
    return Node(
        package=pkg,
        executable=exec,
        name=name,
        output="screen",
        parameters=parameters,
        arguments=arguments,
        condition=condition
        )