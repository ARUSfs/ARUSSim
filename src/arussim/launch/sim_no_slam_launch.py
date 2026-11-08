import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():    
    
    launch_pub = ExecuteProcess(
        cmd=["ros2 topic pub /arussim/launch std_msgs/msg/Bool 'data: true' -1" ],
        shell=True
    )

    return LaunchDescription([
        # Ideal SLAM: publishes /slam/map, /slam/lap_count and TF slam/vehicle from the ARUSSim ground truth
        create_node(pkg='arussim',
                    exec='ideal_slam_exec',
                    name='ideal_slam',
                    config='ideal_slam_config.yaml'),
        # Same nodes and parameters as common_meta/simulation_launch.py, without graph_slam
        create_node(pkg='pc_supervisor',
                    params=[{'mission_name': 'simulation',
                             'sim_mode': False,
                             'record': False}]),
        create_node(pkg='path_planning',
                    params=[{'v_max': 10.0,
                             'ax_max': 7.5,
                             'ay_max': 7.5,
                             'debug': False}]),
        create_node(pkg='controller',
                    params=[{'min_cmd': -22.0,
                             'max_cmd': 22.0,
                             'look_ahead_distance': 6.0,
                             'optimized_look_ahead_distance': 8.0,
                             'use_optimized_trajectory': True,
                             'first_lap_steer_control': "PP",
                             'optimized_steer_control': "MPC",
                             'use_variable_look_ahead_distance': True,
                             'debug': False}]),
        create_node(pkg='visualization'),
        create_node(pkg='can_interface',
                    params=[{'simulation': True}]),
        create_node(pkg='car_state',
                    params=[{'simulation': True,
                             'mission': 'trackdrive',
                             'trackdrive_laps': 1000,
                             'debug': False,
                             'get_arussim_ground_truth': False}]),
        create_node(pkg='trajectory_optimization',
                    params=[{'v_max': 15.0,
                             'd_min': 1.3,
                             'mu_y': 0.8,
                             'mu_throttle': 0.75,
                             'mu_brake': 0.75,
                             'first_lap_optimization_method': "smoothing",
                             'use_ggv': False,
                             'use_grip_estimation': False,
                             'k_trust_lon': 0.8,
                             'k_trust_lat': 0.9,
                             'debug': False}]),
        create_node(pkg='cube_mars_interface',
                    params=[{'simulation': True}]),
        create_node(pkg='grip_estimation'),
        launch_pub
    ])
   

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
        name=name or pkg,
        output="screen",
        parameters=[config_file]+params
        )