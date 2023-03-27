import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory

BRINGUP_PKG = "rr1_bringup"
DESCRIPTION_PKG = "rr1_description"

ROBOT_NAME = "rr1"
URDF_FILE = f"{ROBOT_NAME}.urdf.xacro"
ROBOT_BASE_NAME = "base_link"

CONTROLLER_NAME = "rr1_controller"
CONTROLLER_FILE = f"{CONTROLLER_NAME}.yaml"

RVIZ_CONFIG_FILE = "urdf_viz_file.rviz"
RUN_RVIZ = False


def generate_launch_description():
    # ------------------------------- Fetch paths ------------------------------
    # Package share directories
    description_pkg = get_package_share_directory(DESCRIPTION_PKG)
    bringup_pkg = get_package_share_directory(BRINGUP_PKG)
    urdf_path = os.path.join(description_pkg, "urdf", URDF_FILE)

    robot_description = {"robot_description": Command(['xacro ', urdf_path])}

    #  --------------------------- Instantiate nodes ---------------------------
    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", RVIZ_CONFIG_FILE)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': True}]
    )
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

    nodes = [
        # joint_state_publisher,
        # robot_state_publisher,
        ros_tcp_endpoint,
    ]
    if RUN_RVIZ: nodes.append(rviz)

    return LaunchDescription(nodes)
