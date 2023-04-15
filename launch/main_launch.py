from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

BRINGUP_PKG = "rr1_bringup"

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(BRINGUP_PKG),
                    'launch/gazebo_launch.py'
                ])
            ])
        )

    spawn_rr1_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(BRINGUP_PKG),
                    'launch/spawn_rr1_launch.py'
                ])
            ])
        )

    nodes = [gazebo_launch, spawn_rr1_launch]
    
    return LaunchDescription(nodes)
