#!/usr/bin/env python3

import sys
import argparse

from pydoc import locate
from numpy import size

from nav_msgs.msg import Odometry
import nav_msgs

"""
This script parses a ROS msg type to build an internal model that can easily be iterated through
The functions offered by this script are designed to be used together with the neighbouring script param_parser.py
Running this script directly from the base package directory like phoenix_bridge/msg_parser.py
     (after sourcing ROS) shows an example output of what cog could generate into include/phoenix_bridge/read\write_conversions.h
"""

def get_slots_and_types(a_type: dict):
    slots = []
    slot_types = []
    try:  # If the type cannot be decomposed as a dict, return None
        items = list(a_type.__dict__.items())
    except AttributeError:
        return
    try:
        slots = items[5][1]  # All ROS1 msg types have 6th dict field as __slots__
        slot_types = items[6][1] # and 7th fiels as __slot_types
    except IndexError:
        return

    slots_and_types = {}
    for i in  range(size(slots)):
        slots_and_types[slots[i]] = slot_types[i]

    return slots_and_types


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

    def get_type_name_from_parts(parts):
        """ Helper function. Concatenate non empty strings in a list of strings to form a complete type name """
        name = ""
        for part in parts:
            if part != "":
                name = name + "." + part
        return name[1:]

    def get_subtypes(type_dict:dict, level: int):
        """
        Recursive function to get raw unprocessed subtypes within a dict

        @param type_dict: A dictionary type
        @param level: Used to track the level of recursion. Starting with 1 on top

        @return List of raw tuples which contain the level (i.e. depth), name and type of fields in a ROS msg
        """

        def to_import_format(a_type: str):
            """ Helper function. Converts ros msg format to python import format. Ex: std_msgs/Header to std_msgs.msg.Header """
            parts = a_type.split("/")
            if size(parts) == 1:
                return a_type
            else:
                return parts[0]+".msg."+parts[1]

        subtypes = []
        for key in type_dict:
            subtypes.append((level, key, type_dict[key]))
            type_of_key = locate(to_import_format(type_dict[key]))
            if type_of_key is None or "module" in str(type_of_key): # Base types like int32 and time are core python modules
                continue

            slots_and_types_of_key = get_slots_and_types(type_of_key)
            #if type_of_key is not None:
            subtypes = subtypes + get_subtypes(slots_and_types_of_key, level+1)
        return subtypes

    fields = []
    msg_subtypes = get_subtypes(get_slots_and_types(msg_type), level=1)
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

def get_grpc_type(cpp_type:str):
    """
    Get grpc type for given cpp type by looking up TypesDict defined internally

    @param cpp_type: Type name in cpp
    @return The name of type in grpc

    """

    """ Dictionary to hold type casting from cpp to grpc """
    TypesDict =  {
        "string"  : "CT_String",
        "STRUCT"  : "CT_Struct",
        "float64" : "CT_Real64",
        "double"  : "CT_Real64",
        "int32"   : "CT_Int32",
        "uint32"  : "CT_Uint32",
        "time"    : "CT_IecTime"
        }

    if cpp_type in TypesDict.keys():
        return TypesDict[cpp_type]
    else:
        return cpp_type+"_CASTING_UNDEFINED"

def get_upper_struct(fields, level):
    """ Helper function. From the passed fields list, get the previous upper struct of required level """
    for field in fields[::-1]: # reversed list to look backwards
        if field[1] == level and field[2] == "STRUCT":
            return field[0]

def preview_write_codegen():
    from param_parser import ParamParser, header_format, namespace_format

    params = ParamParser()
    for a_type in params.types_:
        print("----------"+header_format(a_type)+"---------------")
        # Locate does a lexical cast of a string to a discoverable python type
        fields = decompose_ros_msg_type(locate(a_type[0]+".msg."+a_type[1]))
        fields.insert(0,("grpc_object", 0, "STRUCT")) #  Insert the received grpc_object as the uppermost base struct
        print("template <> inline")
        print("void packWriteItem<{}>(::Arp::Plc::Gds::Services::Grpc::WriteItem* grpc_object, {} data_to_pack)"
                .format(namespace_format(a_type),
                        namespace_format(a_type)))

        print("{")
        print("  grpc_object->mutable_value()->set_typecode(::Arp::Type::Grpc::CoreType::CT_Struct);")
        print("")
        for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
            nam = fields[ind][0]
            lvl = fields[ind][1]
            typ = fields[ind][2]

            # Handling special types
            # @TODO: How to handle time properly? CT_XX? set_XXvalue()??
            typ = "double" if typ=="float64" else typ

            var_name = fields[ind][0].replace(".","_")
            grpc_typ = get_grpc_type(fields[ind][2])
            upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

            # Line 1 of boilerplate code
            if upper == "grpc_object":
                print("  ::Arp::Type::Grpc::ObjectType* {} = {}->mutable_value()->mutable_structvalue()->add_structelements();"
                    .format(var_name, upper))
            else:
                print("  ::Arp::Type::Grpc::ObjectType* {} = {}->mutable_structvalue()->add_structelements();"
                    .format(var_name, upper))

            # Line 2 of boilerplate code
            if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                print("  {}->set_typecode(::Arp::Type::Grpc::CoreType::CT_Array);".format(var_name))
            else:
                print("  {}->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(var_name, grpc_typ))

            # Line 3 of boilerplate code
            ## Skipping time for now, @TODO: Handle this
            if typ =="time":
                print("// SKIPPING TIME TYPE FOR NOW")
                print("")
                continue
            if typ != "STRUCT":
                if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                    array_var = var_name+"_array"
                    array_typ = typ.split('[')[0] # Get the type of the array
                    print("  ::Arp::Type::Grpc::TypeArray* {} = {}->mutable_arrayvalue();".format(array_var, var_name))
                    print("  for (auto datum : data_to_pack.{})".format(nam))
                    print("  {")
                    print("    ObjectType* elem = {}->add_arrayelements();".format(array_var))
                    print("    elem->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(get_grpc_type(array_typ)))
                    print("    elem->set_{}value(datum);".format(array_typ))
                    print("  }")
                else:
                    print("  {}->set_{}value(data_to_pack.{});".format(var_name, typ, nam))
            print("")

        print("}")
        print("")

def preview_read_codegen():
    from param_parser import ParamParser, header_format, namespace_format
    params = ParamParser()
    for a_type in params.types_:
        print("----------"+header_format(a_type)+"---------------")
        # Locate does a lexical cast of a string to a discoverable python type
        fields = decompose_ros_msg_type(locate(a_type[0]+".msg."+a_type[1]))
        fields.insert(0,("grpc_object", 0, "STRUCT")) # Insert the received grpc_object as the uppermost base struct
        parent_dict = {}

        print("template <>")
        print("inline void unpackReadObject<{}>(const ObjectType &grpc_object, {}& unpack_to_data)"
                .format(namespace_format(a_type),
                        namespace_format(a_type)))

        print("{")
        for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
            nam = fields[ind][0]
            lvl = fields[ind][1]
            typ = fields[ind][2]

            # Handling special types
            # @TODO: How to handle time properly? CT_XX? set_XXvalue()??
            typ = "double" if typ=="float64" else typ

            var_name = fields[ind][0].replace(".","_")
            grpc_typ = get_grpc_type(fields[ind][2])
            upper = get_upper_struct(fields[:ind], lvl-1) # slice till current index, look for first higher struct

            child_index = 0
            if not upper in parent_dict.keys():
                parent_dict[upper] = 0
            else:
                child_index = parent_dict[upper]
            parent_dict[upper] += 1

            print("  ObjectType {} = {}.structvalue().structelements({});".format(var_name, upper, child_index))
            if typ != "STRUCT":
                if typ=="time":
                    print("  //SKIPPING TIME TYPE FOR NOW")
                    print("")
                    continue
                if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                    array_typ = typ.split('[')[0] # Get the type of the array
                    array_typ = "double" if array_typ=="float64" else array_typ
                    print("  for (int i = 0; i < {}.arrayvalue().arrayelements_size(); i++)".format(var_name))
                    print("  {")
                    print("    unpack_to_data.{}[i] = {}.arrayvalue().arrayelements(i).{}value();".format(nam,var_name, array_typ))
                    print("  }")
                else:
                    print("  unpack_to_data.{} = {}.{}value();".format(nam, var_name, typ))
            print("")

        print("}")
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
