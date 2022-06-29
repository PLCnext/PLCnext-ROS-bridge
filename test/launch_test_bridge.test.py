import os
import sys
import unittest

import ament_index_python

import launch
import launch.actions

import launch_testing
import launch_testing.actions
from launch_testing.asserts import assertSequentialStdout
from launch_ros.actions import Node

import pytest

import rclpy
import yaml

config_bridge = os.path.join(
    ament_index_python.get_package_prefix('phoenix_bridge'),
    'share/phoenix_bridge/config',
    'test_params.yaml'
    )

node_bridge=Node(
    package = 'phoenix_bridge',
    # Do not specify node name, as we spawn multiple nodes from this executable
    executable = 'phoenix_bridge_node',
    output='screen',
    parameters = [config_bridge]
)

@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        node_bridge,
        launch_testing.actions.ReadyToTest(),
    ]), {'node_bridge': node_bridge}

# These tests will run concurrently with the node_bridge.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestGoodProcess(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.node = rclpy.create_node("test_node")

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()

    def test_node_started(self, proc_output):
        """
        Check if node is started within 10 seconds
        """
        proc_output.assertWaitFor('Node starting', timeout=10, stream='stdout', process=node_bridge)


    def test_topics(self, proc_output):
        """
        Check if all the topics defined in the param file have been created
        """
        proc_output.assertWaitFor('Node starting', timeout=10, stream='stdout', process=node_bridge)
        topics_and_types = self.node.get_topic_names_and_types()

        current_topics = []
        for topic in topics_and_types:
            current_topics.append(topic[0])

        with open(config_bridge) as yamlfile:
            params_ = yaml.load(yamlfile, Loader = yaml.FullLoader)

        param_topics = []
        for node in params_:
            param_topics += params_[node]['ros__parameters']['publishers']['topics']
            param_topics += params_[node]['ros__parameters']['subscribers']['topics']

        for topic in param_topics:
            topic = "/"+topic
            assert topic in current_topics
