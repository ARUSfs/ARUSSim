import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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
    create_can0 = ExecuteProcess(
          cmd=['bash', '-c', "ip link show can0 >/dev/null 2>&1 || sudo ip link add dev can0 type vcan"],
            shell=False
    )
    create_can1 = ExecuteProcess(
        cmd=['bash', '-c', "ip link show can1 >/dev/null 2>&1 || sudo ip link add dev can1 type vcan"],
        shell=False
    )

    # Chain processes to run one after another
    sequence = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=modprobe_vcan,
                on_exit=[create_can0]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=create_can0,
                on_exit=[create_can1]
            )
        ),
    ]

    return LaunchDescription([
        modprobe_vcan,
        *sequence, 
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
                    exec='control_sim_exec'),
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
        parameters=[config_file] + params
    )
