import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

 
 
    
def generate_launch_description():
    
    pkg_dir = get_package_share_directory("rrt_explore")

    #Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='robot1',
        description='Host Name / Namespace')

    # Create Launch configuratios
    namespace = LaunchConfiguration('namespace')

    remappings = [('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]
    
    start_exploration_node = Node(
            package='rrt_explore',
            executable='rrt',
            name='rrt',
            namespace=namespace,
            remappings=remappings,
            output="screen",
            parameters=[
            ParameterFile(os.path.join(pkg_dir, 'config', 'params.yaml'), allow_substs=True)],
            # prefix=['xterm -e gdb -ex run --args'],
            # arguments=['--ros-args', '--log-level', 'debug'],
            emulate_tty=True,
        )
    
    start_hostmap_static_tf_publisher = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        name = "hostmap_tf_publisher",
        namespace=namespace,
        remappings=remappings,
        arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", [namespace,"/base_footprint"]])
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_namespace)
    ld.add_action(start_exploration_node)
    # ld.add_action(start_hostmap_static_tf_publisher)

    
    return ld