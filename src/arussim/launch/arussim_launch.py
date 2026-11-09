import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# node:=<name> opens RViz with config/arussim_<name>_rviz_config.rviz
VISUALIZATION_NODES = ['slam']


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'node', default_value='',
            description='Open RViz with the layers of this node (options: '
                        + ', '.join(VISUALIZATION_NODES) + '). Empty: default RViz.'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):

    node = LaunchConfiguration('node').perform(context)
    if node and node not in VISUALIZATION_NODES:
        raise RuntimeError(f"Unknown node '{node}'. Options: {', '.join(VISUALIZATION_NODES)}")

    rviz_config_name = f'arussim_{node}_rviz_config.rviz' if node else 'arussim_rviz_config.rviz'
    rviz_config_file = os.path.join(
        get_package_share_directory('arussim'),
        'config',
        rviz_config_name
    )

    actions = [
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
                    config='supervisor_config.yaml')
    ]

    if node == 'slam':
        # Only what the SLAM layers need: it reads /slam/map, publishes the static TF
        # slam/vehicle -> rslidar and asks graph_slam for the marginal covariances
        actions.append(create_node(pkg='visualization',
                                   exec='slam_visualization_exec',
                                   config='slam_visualization_config.yaml',
                                   name='slam_visualization'))
    return actions


def create_node(pkg, config=None, exec=None, params=[], name=None):

    if config is None:
        config = pkg + "_config.yaml"
    if exec is None:
        exec = pkg + "_exec"

    package_share_directory = get_package_share_directory(pkg)
    config_file = os.path.join(package_share_directory, "config", config)

    return Node(
        package=pkg,
        executable=exec,
        name=name if name is not None else pkg,
        output="screen",
        parameters=[config_file]+params
        )