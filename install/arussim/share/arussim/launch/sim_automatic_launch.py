import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown, IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    track_arg = DeclareLaunchArgument(
        'track',
        default_value='FSG',
        description='Track to simulate'
    )

    event_arg = DeclareLaunchArgument(
        'event',
        default_value='AutoX',
        description='Competition event name (Acceleration, Skidpad, AutoX, etc.)'
    )

    laps_arg = DeclareLaunchArgument(
        'laps_target',
        default_value='10.0',
        description='Number of laps to simulate'
    )

    track = LaunchConfiguration('track')
    event = LaunchConfiguration('event')
    laps_target = LaunchConfiguration('laps_target')

    rviz_config_file = os.path.join(
        get_package_share_directory('arussim'),
        'config',
        'arussim_rviz_config.rviz'
    )
    
    auto_sim_node = create_node(
        pkg='automatic_simulations',
        config='automatic_simulations_config.yaml',
        params=[
            {'event': event,
            'laps_target': laps_target}
        ]
    )

    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('common_meta'),
            '/launch/simulation_launch.py'
        ]),
        condition=IfCondition(
            PythonExpression(["'", track, "' not in ['skidpad','acceleration']"])
        )
    )

    skidpad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('common_meta'),
            '/launch/sim_skidpad_launch.py'
        ]),
        condition=IfCondition(
            PythonExpression(["'", track, "' == 'skidpad'"])
        )
    )

    acceleration_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('common_meta'),
            '/launch/sim_acceleration_launch.py'
        ]),
        condition=IfCondition(
            PythonExpression(["'", track, "' == 'acceleration'"])
        )
    )

    return LaunchDescription([
        track_arg,
        event_arg,
        laps_arg,
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
                    params=[{'vehicle.torque_vectoring': True,
                             'track': track,
                             'csv_state': True,
                             'csv_vehicle_dynamics': True}]),
        create_node(pkg='arussim',
                    exec='sensors_exec',
                    config='sensors_config.yaml'),
        create_node(pkg='arussim',
                    exec='supervisor_exec',
                    config='supervisor_config.yaml',
                    params=[{'csv_supervisor': True}]),
        auto_sim_node,
        simulation_launch,
        skidpad_launch,
        acceleration_launch,
        RegisterEventHandler(
            OnProcessExit(
                target_action=auto_sim_node,
                on_exit=[Shutdown()]
            )
        )
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