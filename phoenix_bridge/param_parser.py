#!/usr/bin/env python3

from numpy import size
import yaml
import os
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class Port:
    topic: str
    datapath: str
    frequency: int

@dataclass
class CommPort:
    address: str
    type: str

@dataclass
class BridgeType:
    node_name: str
    msg_type: str
    grpc: CommPort
    publishers: List[Port]
    subscribers: List[Port]

class ParamParser(object):
    nodes_ = []

    def __init__(self):
        ## Trim subdirectories from path. Needed depending on where this module is called from.
        with open(os.path.join(os.getcwd().replace('/scripts', '').replace('/phoenix_bridge',''), 'phoenix_bridge/config/test_params.yaml')) as yamfile:
            params_ = yaml.load(yamfile, Loader = yaml.FullLoader)
        
        for node in params_:
            pub_topics = params_[node]['ros__parameters']['publishers']['topics']
            pub_datapaths = params_[node]['ros__parameters']['publishers']['datapaths']
            pub_frequencies = params_[node]['ros__parameters']['publishers']['frequencies']
            sub_topics = params_[node]['ros__parameters']['subscribers']['topics']
            sub_datapaths = params_[node]['ros__parameters']['subscribers']['datapaths']
            sub_frequencies = params_[node]['ros__parameters']['subscribers']['frequencies']
            
            if ((size(pub_topics) != size (pub_datapaths)) or (size(pub_topics) != size (pub_frequencies))):
                print(node, "PUBLISHER PARAMS NOT OF EQUAL SIZE!!")
                return

            if ((size(sub_topics) != size (sub_datapaths)) or (size(sub_topics) != size (sub_frequencies))):
                print(node, "SUBSCRIBER PARAMS NOT OF EQUAL SIZE!!")
                return

            publishers = []
            for x in range(size(pub_topics)):
                publishers.append(Port(pub_topics[x], pub_datapaths[x], pub_frequencies[x]))
            
            subscribers = []
            for x in range(size(pub_topics)):
                subscribers.append(Port(sub_topics[x], sub_datapaths[x], sub_frequencies[x]))
                
            self.nodes_.append(BridgeType(node, 
                                   params_[node]['ros__parameters']['msg_type'], 
                                   CommPort(params_[node]['ros__parameters']['grpc']['address'], params_[node]['ros__parameters']['grpc']['type']), 
                                   publishers, 
                                   subscribers))


if __name__ == "__main__":
    obj = ParamParser()
    node: BridgeType
    for node in obj.nodes_:
        print(node.node_name)
        print(" ", node.msg_type)
        print(" ", node.grpc.address, node.grpc.type)
        for pub in node.publishers:
            print("    pub:", pub.topic, pub.datapath, pub.frequency)
        for sub in node.subscribers:
            print("    sub:", sub.topic, sub.datapath, sub.frequency)
        print("")



