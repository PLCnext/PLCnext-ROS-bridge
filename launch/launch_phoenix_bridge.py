import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config_bridge = os.path.join(
        get_package_share_directory('phoenix_bridge'),
        'config',
        'test_params.yaml'
        )

    config_services = os.path.join(
        get_package_share_directory('phoenix_bridge'),
        'config',
        'services_params.yaml'
        )
        
    node_bridge=Node(
        package = 'phoenix_bridge', # Do not specify node name, as we spawn multiple nodes from this executable
        executable = 'phoenix_bridge_node',
        output='screen',
        parameters = [config_bridge]
    )

    node_services=Node(
        package = 'phoenix_bridge', # Do not specify node name, as we spawn multiple nodes from this executable
        executable = 'phoenix_io_services_node',
        output='screen',
        parameters = [config_services]
    )

    ld.add_action(node_bridge)
    ld.add_action(node_services)
    return ld
