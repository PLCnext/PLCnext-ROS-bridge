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

#!/usr/bin/env python3

import sys
import argparse

from pydoc import locate
from numpy import size

"""
This script parses a ROS msg type to build an internal model that can easily be iterated through
The functions offered by this script are designed to be used together with the neighbouring script param_parser.py
Running this script directly from the base package directory like phoenix_bridge/msg_parser.py
     (after sourcing ROS) shows an example output of what cog could generate into include/phoenix_bridge/read\write_conversions.hpp
"""

def decompose_ros_msg_type(msg_type):
    """
    Main entry point to the script to be used to get processed fields of a ros msg.

    @param msg_type: Any ROS msg type. The type class , not the msg object itself.

    @return fields: A list of processed fields, where a field is a 3-tuple which represents every line in a ros msg (name, depth, type)
        name - The name. Depth is appended to struct names to make them unique. Ex: twist_1, header_1, stamp_1 etc..
        depth - In an embedded structure, how far down is this field embedded. Ex: Header.stamp.sec has depth 3.
        type - Type of the field. Can by a base type supported by ros, or a custom "STRCUT" which means it is a struct field and not variable.
            Ex: header has type STRUCT, header.stamp has type STRCUT & header.stamp.sec has type int32

    """
    fields = []
    msg_subtypes = parse_type(msg_type)
    max_depth = 0

    for subtype in msg_subtypes:
        if subtype[0] > max_depth:
            max_depth = subtype[0]

    gentype_name_parts = [""]*max_depth # The complete field name to be generated, represented as a list of its parts

    # Iterate through every subtype, and populate list of fields as either structs or varaibles
    for i in range(len(msg_subtypes)):
        lvl = msg_subtypes[i][0] # field level
        nam = msg_subtypes[i][1] # field name
        typ = msg_subtypes[i][2] # field type

        if typ.find("/") != -1: # If struct ## Empirical observation: all struct types have / in their typenames
            fields.append((nam+"_"+str(lvl), int(lvl), "STRUCT")) # add the struct, append lvl as unique identifier

        gentype_name_parts[lvl-1] = nam # Replace the curent level name part
        for ind in range(lvl, max_depth, 1): # Flush the remaining name parts to the right
            gentype_name_parts[ind] = ""
        # Take the full field name
        if (i < len(msg_subtypes)-1 and msg_subtypes[i][0] >= msg_subtypes[i+1][0] and typ.find("/") == -1) \
            or (i == len(msg_subtypes)-1 and typ.find("/") == -1):
                fields.append((get_type_name_from_parts(gentype_name_parts), int(lvl), typ)) # add the variable
    return fields

""" Dictionary to hold type casting from cpp to grpc """
TypesDict =  {
    "string"  : "CT_String",
    "STRUCT"  : "CT_Struct",
    "float64" : "CT_Real64",
    "double"  : "CT_Real64",
    "int32"   : "CT_Int32",
    "uint32"   : "CT_Uint32"
    }

def get_grpc_type(cpp_type:str):
    """
    Get grpc type for given cpp type by looking up TypesDict defined internally

    @param cpp_type: Type name in cpp
    @return The name of type in grpc

    """
    if cpp_type in TypesDict.keys():
        return TypesDict[cpp_type]
    else:
        return cpp_type+"_CASTING_UNDEFINED"

def parse_type(a_type:dict):
    """
    Parse the ROS msg type directly to extract the underlying structure

    @param a_type: Any dict type. The class and not the object iteslf.
    @return List of the field subtypes within the dictionary
    """
    subtypes = []
    for item in a_type.__dict__.items():
        if item[0] == "_fields_and_field_types":
            subtypes = get_subtypes(item[1], level=1)
    return subtypes

def get_subtypes(type_dict:dict, level: int):
    """
    Recursive function to get raw unprocessed subtypes within a dict

    @param type_dict: A dictionary type
    @param level: Used to track the level of recursion. Starting with 1 on top

    @return List of raw tuples which contain the level (i.e. depth), name and type of fields in a ROS msg
    """
    subtypes = []
    for key in type_dict:
        subtypes.append((level, key, type_dict[key]))
        if locate(to_import_format(type_dict[key])) is not None:
            for item in locate(to_import_format(type_dict[key])).__dict__.items():
                if item[0] == "_fields_and_field_types":
                    subtypes = subtypes + get_subtypes(item[1], level+1)
    return subtypes

def to_import_format(a_type: str):
    """ Helper function. Converts ros msg format to python import format. Ex: std_msgs/Header to std_msgs.msg.Header """
    parts = a_type.split("/")
    if size(parts) == 1:
        return a_type
    else:
        return parts[0]+".msg."+parts[1]

def extract_import_names(typename: str):
    """ Helper function. Extract pymodule name and typename from the parameter header field so that it can be imported """
    parts = typename.split("/")
    libname = ""
    typename = ""
    for i in range(size(parts)):
        if i < size(parts) - 1:
            libname = libname + parts[i] + "."
        else:
            typename = parts[i].title()
    return libname+typename

def get_type_name_from_parts(parts):
    """ Helper function. Concatenate non empty strings in a list of strings to form a complete type name """
    name = ""
    for part in parts:
        if part != "":
            name = name + "." + part
    return name[1:]

def get_upper_struct(fields, level):
    """ Helper function. From the passed fields list, get the previous upper struct of required level """
    for field in fields[::-1]: # reversed list to look backwards
        if field[1] == level and field[2] == "STRUCT":
            return field[0]

def preview_write_codegen():
    import param_parser
    params = param_parser.ParamParser()
    for node in params.nodes_:
        print("----------"+node.header_name+"---------------")
        # locate does a lexical cast from a string to a type that can be found in sys.path
        fields = decompose_ros_msg_type(locate(extract_import_names(node.header_name)))
        fields.insert(0,("grpc_object", 0, "STRUCT")) #  Insert the received grpc_object as the uppermost base struct

        for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
            nam = fields[ind][0]
            lvl = fields[ind][1]
            typ = fields[ind][2]

            var_name = nam.replace(".","_")
            grpc_typ = get_grpc_type(typ)
            upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

            # Line 1 of boilerplate code
            if upper == "grpc_object":
                print("::Arp::Type::Grpc::ObjectType* {} = {}->mutable_value()->mutable_structvalue()->add_structelements();"
                    .format(var_name, upper))
            else:
                print("::Arp::Type::Grpc::ObjectType* {} = {}->mutable_structvalue()->add_structelements();"
                    .format(var_name, upper))

            # Line 2 of boilerplate code
            if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                print("{}->set_typecode(::Arp::Type::Grpc::CoreType::CT_Array);".format(var_name))
            else:
                print("{}->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(var_name, grpc_typ))

            # Line 3 of boilerplate code
            if typ != "STRUCT":
                if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                    array_var = var_name+"_array"
                    array_typ = typ.split('[')[0] # Get the type of the array
                    print("::Arp::Type::Grpc::TypeArray* {} = {}->mutable_arrayvalue();".format(array_var, var_name))
                    print("for (auto datum : data_to_pack.{})".format(nam))
                    print("{")
                    print("  ObjectType* elem = {}->add_arrayelements();".format(array_var))
                    print("  elem->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(get_grpc_type(array_typ)))
                    print("  elem->set_{}value(datum);".format(array_typ))
                    print("}")
                else:
                    print("{}->set_{}value({});".format(var_name, typ, nam))
            print("")

def preview_read_codegen():
    import param_parser
    params = param_parser.ParamParser()
    for node in params.nodes_:
        print("----------"+node.header_name+"---------------")
        # locate does a lexical cast from a string to a type that can be found in sys.path
        fields = decompose_ros_msg_type(locate(extract_import_names(node.header_name)))
        fields.insert(0,("grpc_object", 0, "STRUCT")) #  Insert the received grpc_object as the uppermost base struct

        parent_dict = {}
        for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
            nam = fields[ind][0]
            lvl = fields[ind][1]
            typ = fields[ind][2]

            var_name = nam.replace(".","_")
            upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

            child_index = 0
            if not upper in parent_dict.keys():
                parent_dict[upper] = 0
            else:
                child_index = parent_dict[upper]
            parent_dict[upper] += 1

            print("ObjectType {} = {}.structvalue().structelements({});".format(var_name, upper, child_index))
            if typ != "STRUCT":
                if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                    array_size = typ.split('[')[1].split(']')[0] # Extract size part from expected format xx[xx]
                    if array_size == "": # Variable length arrays do not have size specified in the .msg file, skip handling these
                        # @TODO: Investigate how to parse variable size arrays
                        print("### ARRAY OF UNKNOWN SIZE, SKIPPING")
                    else:
                        print("for (size_t i = 0; i < {}; i++))".format(array_size))
                        print("{")
                        print("  unpack_to_data.{}[i] = {}.arrayvalue().arrayelements(i).doublevalue();"
                            .format(nam,var_name))
                        print("}")
                else:
                    print("unpack_to_data.{} = {}.{}value();".format(nam, var_name, typ))
            print("")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Program designed to be used by cog codegen. Can be run manually to \
                                     preview what the generated output would look like.")
    parser.add_argument("-t", "--type",
                        help="Select which type of codegen output to print: [read, write] ",
                        choices= ["read", "write"], default="read", type= str)
    args = parser.parse_args()

    if args.type == "read":
        preview_read_codegen()
    elif args.type == "write":
        preview_write_codegen()
    else:
        print("Wrong type chosen")
