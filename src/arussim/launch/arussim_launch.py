import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():    
    
    rviz_config_file = os.path.join(
        get_package_share_directory('arussim'),
        'config',
        'arussim_rviz_config.rviz'
    )
    
    return LaunchDescription([
        Node(
            simulation_car = 'ART-25D-2WD-DV',
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        create_node(pkg='arussim',
                    exec='arussim_exec',
                    config='simulator_config.yaml',
                    params=[{'vehicle.torque_vectoring': True}]),
        create_node(pkg='arussim',
                    exec='sensors_exec',
                    config='sensors_config.yaml'),
        create_node(pkg='arussim',
                    exec='supervisor_exec',
                    config='supervisor_config.yaml')
    ])

def read_parameters_file(simulation_car): # quitar, llevar a arussim_node.cpp
    if (simulation_car == 'ART25D-2WD-DV'):
        parameter_file = 'ART-25D-2WD-DV.csv'
    elif (simulation_car == 'ART25D-2WD'):
        parameter_file = 'ART-25D-2WD.csv'
    elif (simulation_car == 'ART25D-4WD-DV'):
        parameter_file = 'ART-25D-4WD-DV.csv'
    elif (simulation_car == 'ART25D-4WD'):
        parameter_file = 'ART-25D-4WD.csv'
    elif (simulation_car == 'ART26D-DV'):
        parameter_file = 'ART-26D-DV.csv'
    else:
        raise ValueError(f"Unknown simulation car: {simulation_car}")    
    parameter_file_path = os.path.join(
        get_package_share_directory('arussim'),
        'resources',
        'parameters'
        parameter_file
    )
    return parameter_file_path
    

def create_node(pkg, config=None, exec=None, params=[]): 

    if config is None:
        config = pkg + "_config.yaml"
    if exec is None:
        exec = pkg + "_exec"

    package_share_directory = get_package_share_directory(pkg)
    config_file = os.path.join(package_share_directory, "config", config)

    return Node(
        package=pkg,
        executable=exec,
        name=pkg,
        output="screen",
        parameters=[config_file]+params
        )