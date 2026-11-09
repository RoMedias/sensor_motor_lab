"""Bridge + Foxglove websocket server.

    ros2 launch sensor_motor_bridge lab.launch.py                       # real board
    ros2 launch sensor_motor_bridge lab.launch.py port:=/tmp/fake_arduino
    ros2 launch sensor_motor_bridge lab.launch.py foxglove:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        Node(package="sensor_motor_bridge", executable="bridge", output="screen",
             parameters=[{"port": LaunchConfiguration("port"),
                          "baud": LaunchConfiguration("baud")}]),
        # Foxglove app connects to ws://localhost:8765
        # (needs: sudo apt install ros-humble-foxglove-bridge)
        Node(package="foxglove_bridge", executable="foxglove_bridge", output="screen",
             parameters=[{"port": 8765}],
             condition=IfCondition(LaunchConfiguration("foxglove"))),
    ])
