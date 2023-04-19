from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

GAZEBO_PKG = "rr1_gazebo"
RVIZ_PKG = "rr1_rviz"

RUN_RVIZ = True

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(GAZEBO_PKG),
                    'launch/gazebo.launch.py'
                ])
            ])
        )

    spawn_rr1_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(GAZEBO_PKG),
                    'launch/spawn_rr1.launch.py'
                ])
            ])
        )
    

    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(RVIZ_PKG),
                    'launch/rviz.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': 'true'}.items()
        )

    nodes = [gazebo_launch, spawn_rr1_launch]
    if RUN_RVIZ:
        nodes.append(rviz)
    
    return LaunchDescription(nodes)
