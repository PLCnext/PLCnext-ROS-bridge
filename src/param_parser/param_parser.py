#!/usr/bin/env python3

import yaml
import os

class ParamParser(object):
    params_ = {}
    keys_ = []
    types_ = []

    def __init__(self):
        with open(os.path.join(os.getcwd().replace('/src/param_parser', ''), 'config/test_params.yaml')) as yamfile:
            params_ = yaml.load(yamfile, Loader = yaml.FullLoader)
        
        for k in params_:
            self.keys_.append(k)
            if k != "communication":
                self.types_.append(k.split("/"))

if __name__ == "__main__":
    obj = ParamParser()
    for type in obj.types_:
        print("{}/{}".format(type[0], type[1]))

