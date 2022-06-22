import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Configuration file for the bridge
    # @todo: Replace this with an argument that the user can provide while launching
    config_bridge = os.path.join(
        get_package_share_directory('phoenix_bridge'),
        'config',
        'test_params.yaml'
        )

    # Configuration file for the io services
    config_services = os.path.join(
        get_package_share_directory('phoenix_bridge'),
        'config',
        'services_params.yaml'
        )
        
    # Main bridge node
    node_bridge=Node(
        package = 'phoenix_bridge', 
        # Do not specify node name, as we spawn multiple nodes from this executable
        executable = 'phoenix_bridge_node',
        output='screen',
        parameters = [config_bridge]
    )

    # IO Services node
    node_services=Node(
        package = 'phoenix_bridge', 
        # Do not specify node name, as we spawn multiple nodes from this executable
        executable = 'phoenix_io_services_node',
        output='screen',
        parameters = [config_services]
    )

    ld.add_action(node_bridge)
    ld.add_action(node_services)
    return ld
