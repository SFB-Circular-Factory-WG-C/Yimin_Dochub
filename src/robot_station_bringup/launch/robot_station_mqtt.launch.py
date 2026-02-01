from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    # Declare arguments
    args = [
        DeclareLaunchArgument('camera_name', default_value='camera'), # Dummy argument
    ]

    # Launch description
    ld = LaunchDescription(
        args 
        + [
            Node(
                package="robot_station_mqtt",
                executable="door_mqtt_to_joint_state",
                name="door_mqtt_to_joint_state",
                remappings=[("/joint_states", "/joint_states_door")],
                parameters=[{'prefix': ''}],
            ),

            Node(
                package="robot_station_mqtt",
                executable="window_mqtt_to_joint_state",
                name="window_mqtt_to_joint_state",
                remappings=[("/joint_states", "/joint_states_window")],
                parameters=[{'prefix': ''}],
            ),

            Node(
                package="robot_station_mqtt",
                executable="joint_states_merger",
                name="joint_states_merger",
            ),
        ]
    )
    return ld
