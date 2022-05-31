#!/usr/bin/env python3

from pydoc import locate
from numpy import size

import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import diagnostic_msgs.msg

### Parse the ROS msg type directly to extract the underlying structure
def parse_type(a_type):
    subtypes = []
    for item in a_type.__dict__.items():
        if item[0] == "_fields_and_field_types":
            subtypes = get_subtypes(item[1], level=1) 
    return subtypes

### Parses a ROS msg object to extract the underlying structure
def parse_msg(msg):
    parsed = []
    for item in type(msg).__dict__.items():
        if item[0] == "_fields_and_field_types":
            parsed = get_subtypes(item[1], level=1) 
    return parsed

### Recursive function to get all subtypes.
### Returns list of tuples which contain the level (i.e. depth), name and type of fields in a ROS msg
def get_subtypes(type_dict, level: int):
    subtypes = []
    for key in type_dict:
        subtypes.append((level, key, type_dict[key]))
        if locate(to_import_format(type_dict[key])) is not None:
            for item in locate(to_import_format(type_dict[key])).__dict__.items():
                if item[0] == "_fields_and_field_types":
                    subtypes = subtypes + get_subtypes(item[1], level+1)
    return subtypes

### Converts from std_msgs/Header to std_msgs.msg.Header
def to_import_format(a_type: str):
    parts = a_type.split("/")
    if size(parts) == 1:
        return a_type
    else:
        return parts[0]+".msg."+parts[1]

### Extract pymodule name and typename from the parameter header field so that it can be imported
def extract_import_names(typename: str):
        parts = typename.split("/")
        libname = ""
        typename = ""
        for i in range(size(parts)):
            if i < size(parts) - 1:
                libname = libname + parts[i] + "."
            else:
                typename = parts[i].title()
        return libname+typename

### Concatenate non empty strings in a list of strings to form a complete type name
def getTypeNameFromParts(parts):
    name = ""
    for part in parts:
        if part != "":
            name = name + "." + part
    return name[1:]

### Decompose the msg type into a list of the fully resolved field names and types
def decomposeRosMsgType(msg_type):
    field_names = []
    msg_subtypes = parse_type(msg_type)
    max_depth = 0
    for subtype in msg_subtypes:
        #print("  "*subtype[0], subtype[1], subtype[2])
        if subtype[0] > max_depth:
            max_depth = subtype[0]

    gentype_name_parts = [""]*max_depth # The complete field name to be generated, represented as a list of its parts
    
    # Iterate through every subtype, and take only the fully resolved field names
    for i in range(len(msg_subtypes)):
        lvl = msg_subtypes[i][0] # field level
        nam = msg_subtypes[i][1] # field name
        typ = msg_subtypes[i][2] # field type
        
        gentype_name_parts[lvl-1] = nam # Replace the curent level name part
        for ind in range(lvl, max_depth, 1): # Flush the remaining name parts to the right
            gentype_name_parts[ind] = ""

        # Take this field name for consideration only if its fully resolved, or is the last part
        if i < len(msg_subtypes)-1: 
            if msg_subtypes[i][0] >= msg_subtypes[i+1][0]: 
                field_names.append((getTypeNameFromParts(gentype_name_parts), typ))
        if i == len(msg_subtypes)-1:
            field_names.append((getTypeNameFromParts(gentype_name_parts), typ))
    return field_names

if __name__ == '__main__':
    import param_parser
    params = param_parser.ParamParser()
    for node in params.nodes_:
        print("----------"+node.header_name+"---------------")
        # locate does a lexical cast from a string to a type that can be found in sys.path
        decomposed_fields = decomposeRosMsgType(locate(extract_import_names(node.header_name)))
        for field in decomposed_fields:
            print("grpc_obj."+field[0], "=", "msg."+field[0], "(type:"+field[1]+")")
