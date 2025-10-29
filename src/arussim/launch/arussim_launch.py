import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():    
    
    rviz_config_file = os.path.join(
        get_package_share_directory('arussim'),
        'config',
        'arussim_rviz_config.rviz'
    )
    modprobe_vcan = ExecuteProcess(
        cmd=['sudo','modprobe', 'vcan'],
        shell=False
    )
    create_can0 = TimerAction(
    period=1.0, 
    actions=[
        ExecuteProcess(
            cmd=['sudo','ip', 'link', 'add', 'dev', 'can0', 'type', 'vcan'],
            shell=False
        )
    ]
    )
    up_can0 = TimerAction(
        period=1.5,
        actions=[
            ExecuteProcess(
                cmd=['sudo','ip', 'link', 'set', 'up', 'can0'],
                shell=False
            )
        ]
    )
    create_can1 = ExecuteProcess(
    cmd=['sudo','ip', 'link', 'add', 'dev', 'can1', 'type', 'vcan'],
    shell=False
    )
    
    up_can1 = ExecuteProcess(
    cmd=['sudo','ip', 'link', 'set', 'up', 'can1'],
    shell=False
    )  


    return LaunchDescription([
        modprobe_vcan,
        create_can0,
        up_can0,
        create_can1,
        up_can1,
        Node(
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
                    config='supervisor_config.yaml'),
        create_node(pkg='arussim',
                    exec='control_sim_exec',
                    config='control_sim_config.yaml'),

    ])


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