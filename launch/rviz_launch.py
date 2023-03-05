import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory

URDF_FILE = "rr1.urdf.xacro"

RVIZ_CONFIG_FILE = "urdf_viz_file.rviz"
BRINGUP_PKG = "rr1_bringup"
DESCRIPTION_PKG = "rr1_description"

def generate_launch_description():
    description_pkg = get_package_share_directory(DESCRIPTION_PKG)
    bringup_pkg = get_package_share_directory(BRINGUP_PKG)
    urdf_path = os.path.join(description_pkg, "urdf", URDF_FILE)

    robot_description = {"robot_description": Command(['xacro ', urdf_path])}
        
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", RVIZ_CONFIG_FILE)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
