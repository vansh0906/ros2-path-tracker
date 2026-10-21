"""Simulation: path publisher + simulated robot + tracker, no Gazebo needed.

ros2 launch path_tracker sim.launch.py controller:=pure_pursuit speed:=0.6 shape:=loop \
    odom_noise:=0.01 csv_path:=/results/pp.csv
The launch ends by itself when the tracker reaches the goal.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('path_tracker'), 'config', 'params.yaml'])
    arg = LaunchConfiguration
    args = [
        DeclareLaunchArgument('controller', default_value='lqr', description='lqr or pure_pursuit'),
        DeclareLaunchArgument('speed', default_value='0.6', description='cruise speed, m/s'),
        DeclareLaunchArgument('shape', default_value='figure8', description='figure8 or loop'),
        DeclareLaunchArgument('odom_noise', default_value='0.0', description='odometry noise, m and rad'),
        DeclareLaunchArgument('csv_path', default_value='', description='where to log the run (empty = no log)'),
    ]
    path_pub = Node(
        package='path_tracker', executable='path_publisher_node', name='path_publisher', output='screen',
        parameters=[params, {'shape': ParameterValue(arg('shape'), value_type=str)}])
    sim = Node(
        package='path_tracker', executable='sim_robot_node', name='sim_robot', output='screen',
        parameters=[params, {'odom_xy_noise': ParameterValue(arg('odom_noise'), value_type=float),
                             'odom_yaw_noise': ParameterValue(arg('odom_noise'), value_type=float)}])
    tracker = Node(
        package='path_tracker', executable='tracker_node', name='tracker', output='screen',
        parameters=[params, {'controller': ParameterValue(arg('controller'), value_type=str),
                             'speed': ParameterValue(arg('speed'), value_type=float),
                             'csv_path': ParameterValue(arg('csv_path'), value_type=str)}])
    stop_all = RegisterEventHandler(OnProcessExit(
        target_action=tracker, on_exit=[EmitEvent(event=Shutdown(reason='tracker reached the goal'))]))
    return LaunchDescription(args + [path_pub, sim, tracker, stop_all])
