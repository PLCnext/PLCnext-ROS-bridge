#!/usr/bin/env python3

from pydoc import locate
from numpy import size

# Dictionary to hold type casting from cpp to grpc
TypesDict = {
    "string"  : "CT_String",
    "STRUCT"  : "CT_Struct",
    "float64" : "CT_Real64",
    "double"  : "CT_Real64",
    "int32"   : "CT_Int32",
    "uint32"   : "CT_Uint32"
    }

# Get grpc type for given cpp type
def get_grpc_type(cpp_type:str):
    if cpp_type in TypesDict.keys():
        return TypesDict[cpp_type]
    else:
        return cpp_type+"_CASTING_UNDEFINED"

### Parse the ROS msg type directly to extract the underlying structure
def parse_type(a_type):
    subtypes = []
    for item in a_type.__dict__.items():
        if item[0] == "_fields_and_field_types":
            subtypes = get_subtypes(item[1], level=1) 
    return subtypes

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
def get_type_name_from_parts(parts):
    name = ""
    for part in parts:
        if part != "":
            name = name + "." + part
    return name[1:]


### Decompose the msg type into a list of fields of the structs and their variables
def decompose_ros_msg_type(msg_type):
    fields = []
    msg_subtypes = parse_type(msg_type)
    max_depth = 0

    for subtype in msg_subtypes:
        #print("  "*subtype[0], subtype[1], subtype[2])
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

# From the passed fields list, get the previous upper struct of required level
def get_upper_struct(fields, level):
    for field in fields[::-1]: # reversed list to look backwards
        if field[1] == level and field[2] == "STRUCT":
            return field[0]

if __name__ == '__main__':
    import param_parser
    params = param_parser.ParamParser()
    for node in params.nodes_:
        print("----------"+node.header_name+"---------------")
        # locate does a lexical cast from a string to a type that can be found in sys.path
        fields = decompose_ros_msg_type(locate(extract_import_names(node.header_name)))
        fields.insert(0,("grpc_object", 0, "STRUCT")) #  Insert the received grpc_object as the uppermost base struct
        print("")
        for ind in range(1, len(fields)): # skip the 0th element, which is the base struct
            nam = fields[ind][0]
            lvl = fields[ind][1] 
            typ = fields[ind][2]

            var_name = fields[ind][0].replace(".","_")
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
                print("//SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW")
            else:
                print("{}->set_typecode(::Arp::Type::Grpc::CoreType::{});".format(var_name, grpc_typ))
            
            # Line 3 of boilerplate code
            if typ != "STRUCT":
                if "[" in typ: # Assuming from empirical evidence that array types have '[' in the type names
                    print("//SKIPPING ARRAY TYPE ASSIGNMENT FOR NOW")
                else:
                    print("{}->set_{}value({});".format(var_name, typ, nam))
            print("")
