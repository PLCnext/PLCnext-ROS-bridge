#!/usr/bin/env python3

###
### Copyright 2022 Fraunhofer IPA
###
### Licensed under the Apache License, Version 2.0 (the "License");
### you may not use this file except in compliance with the License.
### You may obtain a copy of the License at
###
###   http:###www.apache.org/licenses/LICENSE-2.0
###
### Unless required by applicable law or agreed to in writing, software
### distributed under the License is distributed on an "AS IS" BASIS,
### WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
### See the License for the specific language governing permissions and
### limitations under the License.
###

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
    for typ in obj.types_:
        print("----------type", typ,"----------------")
        print("C++ include: {}".format(header_format(typ)))
        print("C++ namespace: {}".format(namespace_format(typ)))
        
