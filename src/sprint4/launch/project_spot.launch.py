import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    # Launch the Gazebo simulation environment
    gazebo_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'champ_config', 'gazebo.launch.py',
            'gui:=false',
            'headless:=True'
        ], 
        shell=True,  # Enables a shell to manage the process more robustly
        output='log'
    )

    # Run the circle detector node
    circle_detector = Node(
        package='sprint4',
        executable='spot_the_circles',
        output='screen'
    )

    # Delay launching navigation to ensure circle_detector starts first
    navigation_launch = TimerAction(
        period=5.0,  # Adjust delay in seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'launch', 'champ_config', 'navigate.launch.py',
                    'rviz:=true'
                ],
                output='log',
                additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': '40'}
            )
        ]
    )

    # Delay launching env_nav to ensure navigation server is ready
    env_nav = TimerAction(
        period=10.0,  # Adjust delay if necessary
        actions=[
            Node(
                package='sprint4',
                executable='spot_navigation',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        circle_detector,
        navigation_launch,
        env_nav,
    ])

