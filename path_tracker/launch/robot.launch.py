"""Real robot: path publisher + tracker only. The robot's own driver provides /odom and takes /cmd_vel.

Start the robot's bringup first (TurtleBot 4, ROSMASTER X3, ...), then:
ros2 launch path_tracker robot.launch.py controller:=lqr speed:=0.25 shape:=loop csv_path:=run1.csv

The path is placed where the robot stands, facing forward, so clear about 2.5 m x 1.5 m in front
and to the LEFT of the robot. Keep a hand on the e-stop. Ctrl+C stops the tracker.
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
        DeclareLaunchArgument('controller', default_value='lqr'),
        DeclareLaunchArgument('speed', default_value='0.25', description='keep it slow on a real robot'),
        DeclareLaunchArgument('shape', default_value='loop'),
        DeclareLaunchArgument('odom_topic', default_value='odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('csv_path', default_value=''),
    ]
    path_pub = Node(
        package='path_tracker', executable='path_publisher_node', name='path_publisher', output='screen',
        parameters=[params, {
            'shape': ParameterValue(arg('shape'), value_type=str),
            'laps': 1,
            'figure8_half_width': 1.0,
            'loop_length': 2.0, 'loop_width': 1.0, 'loop_radius': 0.4,
            'relative_to_robot': True,
            'odom_topic': ParameterValue(arg('odom_topic'), value_type=str)}])
    tracker = Node(
        package='path_tracker', executable='tracker_node', name='tracker', output='screen',
        parameters=[params, {
            'controller': ParameterValue(arg('controller'), value_type=str),
            'speed': ParameterValue(arg('speed'), value_type=float),
            'max_w': 1.0,
            'max_lateral_acc': 0.3,
            'ground_truth_topic': '',
            'odom_topic': ParameterValue(arg('odom_topic'), value_type=str),
            'cmd_vel_topic': ParameterValue(arg('cmd_vel_topic'), value_type=str),
            'csv_path': ParameterValue(arg('csv_path'), value_type=str)}])
    stop_all = RegisterEventHandler(OnProcessExit(
        target_action=tracker, on_exit=[EmitEvent(event=Shutdown(reason='tracker reached the goal'))]))
    return LaunchDescription(args + [path_pub, tracker, stop_all])
