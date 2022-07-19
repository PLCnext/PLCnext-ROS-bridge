#!/usr/bin/env python3

import yaml
import os
import typing

class ParamParser(object):
    params_ = {}
    keys_ = []
    types_ : typing.Tuple[str, str] = []

    def __init__(self):
        with open(os.path.join(os.getcwd().replace('/src/param_parser', ''), 'config/interface_description.yaml')) as yamfile:
            params_ = yaml.load(yamfile, Loader = yaml.FullLoader)
        
        for k in params_:
            self.keys_.append(k)
            if k != "communication":
                self.types_.append(k.split("/"))

def header_format(type: typing.Tuple[str, str]):
    return type[0]+"/"+type[1]

def namespace_format(type: typing.Tuple[str, str]):
    return type[0]+"::"+type[1]

if __name__ == "__main__":
    obj = ParamParser()
    for type in obj.types_:
        print("{}".format(header_format(type)))
