import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory

ROBOT_NAME = "dummy_arm"
URDF_FILE = f"{ROBOT_NAME}.urdf"

def generate_launch_description():
    description_pkg = get_package_share_directory("rr1_description")
    urdf_path = os.path.join(description_pkg, "urdf", URDF_FILE)
    controller_path = os.path.join(description_pkg, "config", "controller.yaml")

    robot_description = {"robot_description": urdf_path}
        
    run_gazebo = ExecuteProcess(
        cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
        output="screen"
    )

    spawn_robot = Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", ROBOT_NAME, "-b", "-file", urdf_path]
            )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        arguments=[urdf_path],
        # parameters=[robot_description]
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

    return LaunchDescription([
        run_gazebo,
        spawn_robot,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        joint_trajectory_controller
    ])
