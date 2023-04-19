from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

RVIZ_PKG = "rr1_rviz"
DESCRIPTION_PKG = "rr1_description"
URDF_FILE = "rr1.urdf.xacro"

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
                    FindPackageShare(DESCRIPTION_PKG), "urdf", URDF_FILE
                ])

    robot_description = {"robot_description": Command(['xacro ', urdf_path])}
        
    joint_state_publisher = Node(
        namespace="rr1",
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace="rr1",
        parameters=[robot_description]
    )

    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(RVIZ_PKG),
                    'launch/rviz.launch.py'
                ])
            ])
        )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
