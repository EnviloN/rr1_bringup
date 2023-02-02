import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

BRINGUP_PKG = "rr1_bringup"
DESCRIPTION_PKG = "rr1_description"
GAZEBO_PKG = "gazebo_ros"

WORLD_FILE = "empty_world.xml"

def generate_launch_description():
    # ------------------------------- Fetch paths ------------------------------
    # Package share directories
    bringup_pkg = get_package_share_directory(BRINGUP_PKG)
    gazebo_pkg = get_package_share_directory(GAZEBO_PKG)

    # Folder with models used in the world description
    world_models = os.path.join(bringup_pkg, 'worlds', 'models')

    # Description package paths for gazebo configuration
    description_pkg_prefix = get_package_prefix(DESCRIPTION_PKG)
    share_folder = os.path.join(description_pkg_prefix, 'share')
    lib_folder = os.path.join(description_pkg_prefix, 'lib')

    # -------------------------- Configure gazebo paths ------------------------
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + share_folder + ':' + world_models
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  share_folder + ':' + world_models

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + lib_folder
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = lib_folder

    print("GAZEBO MODELS PATH: " + str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH: " + str(os.environ["GAZEBO_PLUGIN_PATH"]))
        
    # --------------------- Include gazebo launch description ------------------
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        )
    )  

    launch_argument = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(bringup_pkg, 'worlds', WORLD_FILE), ''],
        description='SDF world file'
    )

    return LaunchDescription([            
            launch_argument,
            launch_gazebo
        ])
