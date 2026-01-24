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
                package="robotiq_2f_urcap_adapter",
                executable="robotiq_2f_adapter_node.py",
                name="robotiq_2f_urcap_adapter",
                output="screen",
                parameters = [{
                    "robot_ip": "172.23.253.44",
                }],
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("robot_station_bringup"), 
                        'launch',
                        'ex-ur10-1.launch.py'
                    ])
                ]),
                launch_arguments={
                    'launch_rviz': 'true',
                    'description_package': 'robot_station_description',
                    'description_file': 'station_ur_orbbec_robotiq2f85.urdf.xacro',
                }.items()
            )
        ]
    )
    return ld
