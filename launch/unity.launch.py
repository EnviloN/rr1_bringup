from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


BRINGUP_PKG = "rr1_bringup"
RVIZ_PKG = "rr1_rviz"

RUN_RVIZ = False


def generate_launch_description():
    #  --------------------------- Instantiate nodes ---------------------------
    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        emulate_tty=True,
        parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
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

    nodes = [
        ros_tcp_endpoint,
    ]
    if RUN_RVIZ: nodes.append(rviz)

    return LaunchDescription(nodes)
