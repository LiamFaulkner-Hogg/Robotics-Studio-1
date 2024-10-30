import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # Set the TurtleBot3 model environment variable
    os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

    # Launch the Gazebo simulation environment
    gazebo_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'no_roof_small_warehouse.launch.py'],
        output='log'
    )

    # Run the circle detector node
    circle_detector = Node(
        package='sprint4',
        executable='circle_detector',
        output='screen'
    )

    # Delay launching navigation to ensure circle_detector starts first
    navigation_launch = TimerAction(
        period=5.0,  # Adjust delay in seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                    'map:=' + os.path.expanduser('~/map.yaml')
                ],
                output='log',  # Only show warning and error logs
                additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': '40'}  # Set log level to WARN
            )
        ]
    )

    env_nav = Node(
        package='sprint4',
        executable='EnvNavigation',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        circle_detector,
        navigation_launch,
        env_nav,
    ])
