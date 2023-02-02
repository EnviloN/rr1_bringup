import os
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory

BRINGUP_PKG = "rr1_bringup"
DESCRIPTION_PKG = "rr1_description"
GAZEBO_PKG = "gazebo_ros"

ROBOT_NAME = "rr1"
URDF_FILE = f"{ROBOT_NAME}.urdf.xacro"
ROBOT_BASE_NAME = "base_link"

CONTROLLER_NAME = "rr1_controller"
CONTROLLER_FILE = f"{CONTROLLER_NAME}.yaml"

RVIZ_CONFIG_FILE = "urdf_viz.rviz"
RUN_RVIZ = True

ROBOT_POSITION = [0.0, 0.0, 0]
ROBOT_ORIENTATION = [0.0, 0.0, 0.0]


def generate_launch_description():
    # ------------------------------- Fetch paths ------------------------------
    # Package share directories
    bringup_pkg = get_package_share_directory(BRINGUP_PKG)
    description_pkg = get_package_share_directory(DESCRIPTION_PKG)
    urdf_path = os.path.join(description_pkg, "urdf", URDF_FILE)
    controller_path = os.path.join(description_pkg, "config", CONTROLLER_FILE)

    robot_description = {"robot_description": Command(['xacro ', urdf_path])}

    #  --------------------------- Instantiate nodes ---------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])}]
    )

    rviz_config = os.path.join(bringup_pkg, "rviz", RVIZ_CONFIG_FILE)
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config]
    )

    entity_name = ROBOT_NAME + "-" + str(int(random.random()*100000))
    spawn_robot = Node(
        package=GAZEBO_PKG,
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity', entity_name,
            '-x', str(ROBOT_POSITION[0]), '-y', str(ROBOT_POSITION[1]), '-z', str(ROBOT_POSITION[2]),
            '-R', str(ROBOT_ORIENTATION[0]), '-P', str(ROBOT_ORIENTATION[1]), '-Y', str(ROBOT_ORIENTATION[2]),
            '-topic', '/robot_description'
        ]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_path],
        output={
            "stdout": "screen",
            "stderr": "screen"
        }
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]
    )

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"]
    )

    nodes = [
        robot_state_publisher,
        spawn_robot,
        controller_manager,
        joint_state_broadcaster,
        joint_trajectory_controller
    ]
    if RUN_RVIZ: nodes.append(rviz)

    return LaunchDescription(nodes)
