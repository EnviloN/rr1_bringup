import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

URDF_FILE = "dummy_arm.urdf"

def generate_launch_description():
    description_pkg = get_package_share_directory("rr1_description")
    urdf_path = os.path.join(description_pkg, "urdf", URDF_FILE)
        
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        arguments=[urdf_path]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log"
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz
    ])
