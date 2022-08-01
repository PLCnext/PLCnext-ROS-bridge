# PHOENIX BRIDGE

ROS package to bridge ROS Noetic to PLCnext devices. Requires the dependencies from the PLC SDK to be properly installed in order to compile and function. Refer [Phoenix Dependencies](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_dependencies/-/tree/noetic) repo for info on this.

The main principle is that this package can parse an interface description file and generate the code required for setting up bridging topics for the ROS msg types as specified.

- [PHOENIX BRIDGE](#phoenix-bridge)
  - [Main features](#main-features)
    - [The Interface Description File (IDF)](#the-interface-description-file-idf)
    - [Generation of specified code](#generation-of-specified-code)
    - [Launching the bridge](#launching-the-bridge)
    - [IO Services](#io-services)
    - [Parsers](#parsers)
    - [Liveliness Check](#liveliness-check)
  - [Requirements](#requirements)
  - [Usage](#usage)
    - [From Source](#from-source)
  - [Developer Guide](#developer-guide)
    - [Code generation with Cog](#code-generation-with-cog)
    - [Scaling codegen for more message types](#scaling-codegen-for-more-message-types)
    - [Linting](#linting)
    - [Unit Testing](#unit-testing)
    - [Component Testing](#component-testing)
    - [Integration Testing](#integration-testing)
  - [CI/CD](#cicd)

## Main features
### The Interface Description File (IDF)

The [Interface description file](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_bridge/-/blob/foxy/devel/config/interface_description.yaml), `config/interface_description.yaml`, is a yaml format file structured as a ROS parameter file. This is where the user defines which bridging topics are to be created. Prameters in ROS2 are clustered together per node, and each node in from this package is dedicated to bridging one ROS2 msg type. 

* The primary entries are always node names. For each node, the ROS2 parameters are declared under the namespace `ros__parameters`.
* The first parameter is alwas `grpc` and requires the `grpc.address` param to be set to the correct value to communicate with the grpc server on the PLC. A typical value is `unix:/run/plcnext/grpc.sock`.
* The second parameter is always `msg_type`. This dictates which ROS2 msg type name the node should bridge.
* The next two parameters `publishers` and `subscribers` are optional. These are required if publishing and subscribing topics respectively are to be created for the type of this node. If one or both are omitted, then those type of topics are simply not generated at launch time. Both of them have 3 list type sub-parameters, :
    * `topics` [list of strings]- the names of the topics that should be created
    * `datapaths` [list of strings]- the instance paths to the variable in the global dataspace of the PLC corresponding per index position to the above topics.
    * `frequencies`[list of integers]- the frequencies corresponding per index position to the above topics at which data on the topic is updated.

> NOTE: **ALL 3 OF THESE LISTS MUST HAVE THE SAME LENGTH, SINCE THE TOPICS DATA CORRESPOND PER INDEX**

> NOTE: *Publishers [PLC -> ROS]*: Get data from PLC and publish on ROS topics.

> NOTE: *Subscribers [ROS -> PLC]*: Get data from ROS topics and send to PLC.

Consider an example parameter definition in this file:

```yaml
odom_bridge:
  ros__parameters:
    grpc:
      address: "unix:/run/plcnext/grpc.sock"
      type: "tcp"
    msg_type: "nav_msgs/msg/Odom" # Include header filename is derived from this in lower case. Scope resolution is derived from this as well.
    header_name: "nav_msgs/msg/odometry" # Optional if C++ header include filename is different from msg type name. Do not include the ".hpp" part
    publishers: # The next 3 lists must have the same size. Elements of the same index from each list characterise one port.
      topics: [pub_odom_1, pub_odom_2]
      datapaths: [Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data, Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data]
      frequencies: [11, 12]
    subscribers:
      topics: [sub_odom_1, sub_odom_2]
      datapaths: [Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data, Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data]
      frequencies: [13, 14]
```
Explanation:   
`odom_bridge:` - This instructs the a node called `odom_bridge` is to be generated.   

`grpc.address: "unix:/run/plcnext/grpc.sock"` - The node shall use this value as the grpc address for communication with the PLC.

`msg_type: "nav_msgs/msg/Odom" ` - It shall handle bridging `nav_msgs/msg/Odom` type ROS2 messages. 
`header_name: "nav_msgs/msg/odometry"` - However, the C++ incldue header name is different and is actually `nav_msgs/msg/odometry.hpp`.

`publishers:` -  It shall spawn some publishers.    

`topics: [pub_odom_1, pub_odom_2]` - Two topics are to be created, with the names `pub_odom_1` and `pub_odom_2`   

`datapaths: [Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data, Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data]` - `pub_odom_1` shall connect to path `Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data`, and `pub_odom_2` shall connect to path `Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data`   

`frequencies: [11, 12]`- `pub_odom_1` shall have 11Hz frequency, and `pub_odom_1` 12Hz

```yaml
subscribers:
      topics: [sub_odom_1, sub_odom_2]
      datapaths: [Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data, Arp.Plc.Eclr/MainInstance.gRPC_Obj.odom_data]
      frequencies: [13, 14]
```
Similarly 2 subscriber topics with this specified data are to be created.

> NOTE: The IDF is actually read twice, first during compilation of source code, and second again when the bridge is launched. Changes in the IDF between these two steps can lead to different behavior, details below. Ideally, such changes should be avoided.

### Generation of specified code

Based on the specification of the Interface Description File, sections of the bridge code shall be generated, which implement factories required to synthesize topics of the specified ROS msg types. In this phase, the actual numbers, names and instance paths of topics are not important, just the ROS msg types. One factory is generated per each specified ROS msg type.

Code generation is automatically triggered when this package is built. See the developer section below for more details.

### Launching the bridge

When the bridge is actually launched, the IDF is loaded again. As described in the above section, the bridge already has some factories generated for specific ROS msg types, and only the topic details for these types are now loaded. Based on these parameters, the factories synthesis ROS topics.

Each topics functions on a standalone thread to send/get data between PLC and ROS. You can get data from the publishers and send data through subscribers.

### IO Services

This package also provides some static services to set/get digital IOs:

* `single_set_IO`: Set a single boolean digital IO to true or false. Specify the instance path of the IO and the value to set.
* `single_get_IO`: Get the value of a single boolean digital IO. Specify the instance path.
* `batch_get_IO`: Get the values of several boolean digital IOs. Specify a list of datapaths.
* `batch_set_IO`: Set the values of several boolean digital IOs. Specify a list of datapahs and values.
* `write_analog_IO`: Set the value of an analog IO.  Specify the instance path of the IO and the value to set.
* `read_analog_IO`: Get the value of an analog IO. Specify the instance path of the IO .

### Parsers

There are two main parsing python scripts developed in this package. These parsers are used in code generation, and are hardcoded to use the IDF in the default directory only - `config/interface_description.yaml`.

* `phoenix_bridge/param_parser.py`: Offers functions to read the contents of the yaml file to extract the ROS msg types that need to be bridged. Used in all generated sections.
* `phoenix_bridge/msg_parser.py`: Offers functions that parse ROS msg types to extract their underlying dictionary structure. Used mainly to generate conversion functions to transform data between grpc type and ROS msg type.

### Liveliness Check

The bridge packs in a [livliness check node](src/liveliness_check.cpp), that pings a variable on the PLC at a determined frequency in a handshake fashion and shouts a warning if check fails. All parameters are [configurable](config/liveliness_config.yaml).

## Requirements

Install all ROS related requirements by running the following command from the workspace directory:    
`rosdep update && rosdep install --from-paths src --ignore-src -y`

Install the non-ROS python requirements:   
`pip3 install -r src/phoenix_bridge/requirements.txt`

Install PLC related requirements by referring to the latest guidelines from Phoenix Contact. Optionally, use a pre-installed docker image.

## Usage
### From Source

This package can be compiled from source and launched to setup the bridge. This must be done from a docker image with correctly installed dependencies running on the PLC to fully function.

* Update the IDF and specify which topics are needed, clustered by msg types (See above for more info)
* Build the package from the workspace directory after sourcing ROS: `colcon build --symlink-install --event-handlers console_direct+`
* Source the workspace: `source install/local_setup.bash`
* Launch the bridge to create topics & IO services: `ros2 launch phoenix_bridge launch_phoenix_bridge.py`

>NOTE: Before launching the bridge, you can update the IDF to change the topic details i.e. change the contents of the `publishers` and `subscribers` list parameters of each type. You can even remove some types or topics that are not needed, but if you add more ROS msg types, you must recompile the package.

## Developer Guide

### Code generation with Cog

Using the following python3 package for code generation: [cog](https://nedbatchelder.com/code/cog/index.html#h_installation)

Read the types mentioned in the param file that the user wants to bridge -> generate the required code to spawn these bridges.

> NOTE: The codegen scripts currently only handle fixed length arrays inside ros messages, and skip variable length arrays

CMakeLists.txt has a [custom target](https://github.com/ipa-kut/phoenix_bridge/blob/f3cf41a185e6ff1fad2fad41b6b9e6fd9a187c0c/CMakeLists.txt#L6) to generate the code before the package is built.

Cog can also be manually run by first navigating to the package directory in a terminal and running the command Ex:
```
$ cd <workspace_folder>/src/phoenix_bridge
$ cog -r include/phoenix_bridge/write_conversions.hpp
```

Tests can be manually run as follows:
1. Build the project: `colcon build --symlink-install`
2. Run tests: `colcon test`
3. See results: `colcon test-result --verbose`

Tests shall also be run by CI automatically when code is pushed to gitlab.

### Scaling codegen for more message types

The main infrastructure to parse ROS2 message types and generate code is in place. However, when scaling up this bridge to parse more message types, there are some parts of the code that might need to be expanded.

1. Type name differences - Some type names are inconsistent between the [ROS2 IDL](https://design.ros2.org/articles/idl_interface_definition.html) and protobuf/grpc. For example, IDL denotes `boolean` whereas protobuf denotes `bool`. Since codegen is templated by strings, this could break the code. Each such exceptional case has to be handled individually, and is currently done in the function `phoenix_bridge/msg_parser.py/decompose_ros_msg_type()/#Line65` - [relative_link](phoenix_bridge/msg_parser.py#L65). The target is to change IDL to match protobuf.

2. Type casting - The protobuf types need to be cast into [IEC types](https://en.wikipedia.org/wiki/IEC_61131-3#Data_types), but more specifically, the types supported by PLCnext as seen in `ArpTypes.pb.h/CoreType` - [relative_link](include/phoenix_bridge/ServiceStubs/ArpTypes.pb.h#L81). This casting is handled in the [msg_parser](phoenix_bridge/msg_parser.py#L89) through the `TypesDict` dictionary. This dictionary has a few type castings already defined, but is not an exhaustive list. When new types are encountered which are undefined, the message `<type>_CASTING_UNDEFINED` can be seen in the generated code. To avoid/resolve this, `TypesDict` has to be extended by defining more casting pairs. The target is to cast from IDL type to PLCnext type.

3. Type linking - In order to compile the package properly, when new ROS2 msg libraries are included, CMakeLists.txt should also be extended to include these so they can be linked. This is done by adding the type library to the variable [MSG_TYPES_TO_LINK](CMakeLists.txt#L37). Furthermore, `package.xml` should also be extended to add dependencies to the new libraries [here](package.xml#L23). This linking list is currently kept to the minimum used, in order to avoid linking every ROS type and bloating the package too much.

### Linting

Only the handwritten non-generated files of this project should be linted, and the generated files should be excluded. The include files are specified in the CMakeLists variable `files_to_lint` in the `if(BUILD_TESTING)` section. For this reason, auto linting is not used.

Linting is done when package tests are invoked, either manually from terminal as described above, or by CI. Individual linters can also be invoked manually from terminal, see details below. First install the ament dependencies by running rosdep install in the workspace containing this package: `rosdep update && rosdep install --from-path src --ignore-src -y`

Linters added:

1. `ament_cmake_clang_format`: Checks for clang formatting. Errors can be resolved by invoking the following command from terminal for the file/files/directory with the formatting errors:
    * `ament_clang_format --reformat <file/files/directory>`

2. `ament_cmake_cpplint`: Runs cpplint [static code checker](https://www.google.com/search?channel=fs&client=ubuntu&q=cpplint). To manually find linting errors, run from terminal:
    * `ament_cpplint --filters=-legal/copyright,-build/include_order <file/files/directory>`
    * build/include_order is excluded from here since clang_format does it also, and can sometimes collide

3. `ament_cmake_cppcheck`: Runs cppcheck [static code analyzer](https://cppcheck.sourceforge.io/). To manually find linting errors, run from terminal:
    * `ament_cppcheck <file/files/directory>`

4. `ament_cmake_lint_cmake`: Lints CMakeLists.txt. To manually find linting errors, run from terminal in the package directory:
    * `ament_lint_cmake`

4. `ament_cmake_xmllint`: Lints xml files like package.xml. To manually find linting errors, run from terminal in the package directory:
    * `ament_xmllint`

### Unit Testing

Unit tests are added using [gtest](https://github.com/google/googletest) under the `test` folder and declared in cmake as well.

Unit tests added:

1. `test_write_conversions.cpp`: Test the write function `packWriteItem()` available in `include/phoenix_bridge/write_conversions.hpp`.   
2. `test_read_conversions.cpp`: Test the read function `unpackReadObject()` available in `include/phoenix_bridge/read_conversions.hpp`.   

### Component Testing

Component tests are added using [launch_testing]() under the `test`folder and declared in cmake as well.

Component tests added:

1. `launch_test_bridge.test.py`: Launches the bridge node using the same config as `launch/launch_phoenix_bridge.py` and runs some tests -
    * `test_node_started()` - Check if the node is correctly started.
    * `test_topics()` - Checks the the bridge creates all topics as specified.

### Integration Testing

Automated integration testing is not yet implemented, but to help manaul testing, some scripts are provided under `test/scripts` to publish random data on default topics.

## CI/CD

Consists of 3 phases:

1. Build and test the package using [industrial_ci](https://github.com/ros-industrial/industrial_ci). This runs all tests as configured in CMakeLists.txt

2. Doxygen documentation generation. The generated HTML documentation is available as downloadable artefacts and can also be served externally.

3. Dockerization builds docker images ONLY FOR TAGGED COMMITS. The commit tag is chosen as the tag for the image as well (make sure to update package.xml as well). Therefore, in order to trigger CD and build the images, tag the commit with the version of the package and push with tags. The created images are available in the [gitlab container repository.](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_bridge/container_registry/)
