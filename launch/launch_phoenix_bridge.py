import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('phoenix_bridge'),
        'config',
        'test_params.yaml'
        )
        
    node=Node(
        package = 'phoenix_bridge', # Do not specify node name, as we spawn multiple nodes from this executable
        executable = 'phoenix_bridge_node',
        parameters = [config]
    )

    ld.add_action(node)
    return ld