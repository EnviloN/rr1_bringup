import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory

BRINGUP_PKG = "rr1_bringup"

ROBOT_NAME = "rr1"
URDF_FILE = f"{ROBOT_NAME}.urdf.xacro"
ROBOT_BASE_NAME = "base_link"

CONTROLLER_NAME = "rr1_controller"
CONTROLLER_FILE = f"{CONTROLLER_NAME}.yaml"

RVIZ_CONFIG_FILE = "urdf_viz_file.rviz"
RUN_RVIZ = True


def generate_launch_description():
    # ------------------------------- Fetch paths ------------------------------
    # Package share directories
    bringup_pkg = get_package_share_directory(BRINGUP_PKG)
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

    # forward_position_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["--stopped", "forward_position_controller", "--controller-manager", "/controller_manager"]
    # )

    # joint_trajectory_controller = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"]
    # )

    # forward_position_controller_event = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster,
    #         on_exit=[forward_position_controller]
    #     )
    # )

    # joint_trajectory_controller_event = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster,
    #         on_exit=[joint_trajectory_controller]
    #     )
    # )

    nodes = [
        # forward_position_controller_event,
        # joint_trajectory_controller_event,
        ros_tcp_endpoint,
        # spawn_robot,
    ]
    if RUN_RVIZ: nodes.append(rviz)

    return LaunchDescription(nodes)
