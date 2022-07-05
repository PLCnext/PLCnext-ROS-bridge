#!/usr/bin/env python3

"""
This script extracts parameters from the param file and stores them in an internal model that can be easily traversed.
Designed to be used by cog to generate C++ code at build time.
Running this script directly from the base package directory like phoenix_bridge/param_parser.py
     (after sourcing ROS) shows the model of the param file that is built
"""

from numpy import size
import yaml
import os
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Port:
    """ Define the struct for a Port """
    topic: str
    datapath: str
    frequency: int

@dataclass
class CommPort:
    """ Define the struct for a Cummunication Port """
    address: str
    type: str

@dataclass
class BridgeType:
    """ Define the struct for a BridgeType """
    node_name: str
    msg_type: str
    header_name : str
    grpc: CommPort
    publishers: List[Port]
    subscribers: List[Port]


def getResolvedTypeName(typename: str):
        """ Converts the ROS msg typename into a fully scope resolved C++ class name """
        parts = typename.split("/")
        result = ""
        for i in range(size(parts)):
            if i < size(parts) - 1:
                result = result + parts[i] + "::"
            else:
                result = result + parts[i].title()
        return result

class ParamParser(object):
    """
    Class to access parameter parsing functionality. Constructor does parsing and saves resulting model in varialbe nodes_.
    nodes_ is a list of the bridge types found.
    @todo: Find a way to pass the param file as an argument instead. Challenging to do with cog at build time.
    """
    nodes_ = []

    def __init__(self):
        """ Parse the param file from a hardcoded path. Save resulting model in nodes_ variable """
        ## Trim subdirectories from path. Needed depending on where this module is called from.
        with open(os.path.join(os.getcwd().replace('/scripts', '').replace('/phoenix_bridge',''), 'phoenix_bridge/config/test_params.yaml')) as yamfile:
            params_ = yaml.load(yamfile, Loader = yaml.FullLoader)

        for node in params_:
            # Safe get ros__parameters
            if "ros__parameters" not in params_[node]:
                print(node, ": Improperly formatted ros2 param yaml, must contain field ros__parametrs!! Exiting")
                return
            rosparams = params_[node]['ros__parameters']

            # Safe get msg_type and header_name rosparam
            if "msg_type" not in rosparams:
                print(node, ": msg_type not defined as a rosparam!! Exiting")
                return
            msg_type = rosparams['msg_type']

            if "header_name" in rosparams:
                header_name = rosparams['header_name']
            else:
                header_name = rosparams['msg_type'].lower()

            # Safe get grpc params
            if "grpc" not in rosparams or "address" not in rosparams['grpc']:
                print(node, ": grpc.address not defined under rosaparams!! Exiting")
                return
            grpc_address = rosparams['grpc']['address']
            grpc_type = rosparams['grpc']['type'] if "type" in rosparams['grpc'] else ""

            # Safe get publisher params
            publishers = rosparams['publishers']  if "publishers" in rosparams else dict()
            if publishers is not None:
                pub_topics = publishers['topics'] if "topics" in publishers else []
                pub_datapaths = publishers['datapaths'] if "datapaths" in publishers else []
                pub_frequencies = publishers['frequencies'] if "frequencies" in publishers else []
                if ((size(pub_topics) != size (pub_datapaths)) or (size(pub_topics) != size (pub_frequencies))):
                    print(node, "PUBLISHER PARAMS NOT OF EQUAL SIZE!! Exiting")
                    return
            else:
                pub_topics = pub_datapaths = pub_frequencies = []

            # Safe get subscriber params
            subscribers = rosparams['subscribers'] if "subscribers" in rosparams else dict()
            if subscribers is not None:
                sub_topics = subscribers['topics'] if "topics" in subscribers else []
                sub_datapaths = subscribers['datapaths'] if "datapaths" in subscribers else []
                sub_frequencies = subscribers['frequencies'] if "frequencies" in subscribers else []
                if ((size(sub_topics) != size (sub_datapaths)) or (size(sub_topics) != size (sub_frequencies))):
                    print(node, "SUBSCRIBER PARAMS NOT OF EQUAL SIZE!! Exiting")
                    return
            else:
                sub_topics = sub_datapaths = sub_frequencies = []

            publishers_list = []
            for x in range(size(pub_topics)):
                publishers_list.append(Port(pub_topics[x], pub_datapaths[x], pub_frequencies[x]))

            subscribers_list = []
            for x in range(size(sub_topics)):
                subscribers_list.append(Port(sub_topics[x], sub_datapaths[x], sub_frequencies[x]))

            self.nodes_.append(BridgeType(node,
                                   msg_type,
                                   header_name,
                                   CommPort(grpc_address, grpc_type),
                                   publishers_list,
                                   subscribers_list))


if __name__ == "__main__":
    obj = ParamParser()
    node: BridgeType
    for node in obj.nodes_:
        print(node.node_name)
        print(" Msg type-", node.msg_type)
        print(" C++ resolved class-", getResolvedTypeName(node.msg_type))
        print(" C++ include header-", node.header_name,)
        print(" gRPC channel- ", node.grpc.address, node.grpc.type)
        print(" Ports-")
        for pub in node.publishers:
            print("    pub:", pub.topic, pub.datapath, pub.frequency)
        for sub in node.subscribers:
            print("    sub:", sub.topic, sub.datapath, sub.frequency)
        print("")
