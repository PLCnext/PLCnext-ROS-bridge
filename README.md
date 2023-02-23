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
    - [Testing](#testing)
    - [Unit Testing](#unit-testing)
    - [Component Testing](#component-testing)
  - [CI/CD](#cicd)

## Main features
### The Interface Description File (IDF)

The [Interface description file](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_bridge/-/blob/noetic/grpc/config/interface_description.yaml), `config/interface_description.yaml`, is a yaml format file structured as a ROS parameter file. This is where the user defines which bridging topics are to be created. Topics are clustered together based on their msg types and as publishers or subscribers.

* The first main parameter must be parameters for the grpc client, and requires the `communication.grpc.address` param to be set to the correct value to communicate with the grpc server on the PLC. A typical value is `unix:/run/plcnext/grpc.sock`.
* Every main parameter after this should be a ROS msg type, and informs the bridge that this specified type should be bridged. Ex: `std_msgs/String`, `geometry_msgs/Twist` etc...
* Under the specified type parameter, two sub parameters are expected - `publishers` and `subscribers`. These are lists of 3-Tuples, where each tuple shall follow the format `[topic_name, instance_path, frequency]`.
    * `topic_name` [string value] is the name of the topic that should be created (as either a publisher or a subscriber)
    * `instance_path` [string value ]is the path to the variable in the global dataspace of the PLC to which this topic connects. A typical value follows the pattern `Arp.Plc.Eclr/<MainInstance_name>.<object_name>.<field_name>`.
    * `frequency`[integer value] defines the frequency at which data on the topic is updated. Only applicable to publishers.

> NOTE: **Publishers [PLC -> ROS]**: Get data from PLC and publish on ROS topics.

> NOTE: **Subscribers [ROS -> PLC]**: Get data from ROS topics and send to PLC.

Consider an example parameter definition in this file:

```yaml
geometry_msgs/Twist:
  publishers:
    - [pub_twist_1, datapath_twist_1, 30]
    - [pub_twist_2, datapath_twist_2, 40]
  subscribers:
    - [sub_twist_1, datapath_twist_3, 30]
    - [sub_twist_2, datapath_twist_4, 40]
```
This instructs the bridge that `geometry_msgs/Twist` type topics are to be created.

* With this type, two publishers are to be created, the first of which shall be called `pub_twist_1` and map to the instance path on the PLC `datapath_twist_1` and publish at a rate of 30Hz. The second publisher shall be called `pub_twist_2`, connect to `datapath_twist_2` and publish at 40 Hz.
* Furthermore, two subscribers are to be created, namely `sub_twist_1` and `sub_twist_2`, with their connected instance paths being `datapath_twist_3` and `datapath_twist_3` respectively. In the current version, frequency has no impact.

> NOTE: The IDF is actually read twice, first during compilation of source code, and second again when the bridge is launched. Changes in the IDF between these two steps can lead to different behavior, details below. Ideally, such changes should be avoided.

### Generation of specified code

Based on the specification of the Interface Description File, sections of the bridge code shall be generated, which implement factories required to synthesize topics of the specified ROS msg types. In this phase, the actual numbers, names and instance paths of topics are not important, just the ROS msg types. One factory is generated per each specified ROS msg type.

Code generation is automatically triggered when this package is built. See the developer section below for more details.

### Launching the bridge

When the bridge is actually launched, the IDF is loaded again. As described in the above section, the bridge already has some factories generated for specific ROS msg types, and only the topic details for these types are now loaded. Based on these parameters, the factories synthesis ROS topics.

Each topics functions on a standalone thread to send/get data between PLC and ROS. You can get data from the publishers and send data through subscribers.

### IO Services

This package also provides some static services to set/get digital IOs:

* `single_set_io`: Set a single boolean digital IO to true or false. Specify the instance path of the IO and the value to set.
* `single_get_io`: Get the value of a single boolean digital IO. Specify the instance path.
* `batch_get_io`: Get the values of several boolean digital IOs. Specify a list of datapaths.
* `batch_set_io`: Set the values of several boolean digital IOs. Specify a list of datapahs and values.

### Parsers

There are two main parsing python scripts developed in this package. These parsers are used in code generation, and are hardcoded to use the IDF in the default directory only - `config/interface_description.yaml`.

* `src/parsers/param_parser.py`: Offers functions to read the contents of the yaml file to extract the ROS msg types that need to be bridged. Used in all generated sections.
* `src/parsers/msg_parser.py`: Offers functions that parse ROS msg types to extract their underlying dictionary structure. Used mainly to generate conversion functions to transform data between grpc type and ROS msg type.

### Liveliness Check

The bridge also contains a [Liveliness Check node](src/liveliness_check.cpp) that periodically pings the PLC with a handshake mechanism to check if the connection is good. The variable to ping and timing can be [configured](config/liveliness_config.yaml).

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
* Build the package from the workspace directory: `catkin_make`
* Source the workspace: `source devel/setup.bash`
* Launch the bridge to create topics: `roslaunch phoenix_bridge phoenix_bridge.launch`
* Launch the services to get/set/read/write IOs: `roslaunch phoenix_bridge phoenix_io_services.launch`

>NOTE: Before launching the bridge, you can update the IDF to change the topic details i.e. change the contents of the `publishers` and `subscribers` list parameters of each type. You can even remove some types or topics that are not needed, but if you add more ROS msg types, you must recompile the package.

## Developer Guide

### Code generation with Cog

Using the following python3 package for code generation: [cog](https://nedbatchelder.com/code/cog/index.html#h_installation)

Read the types mentioned in the param file that the user wants to bridge -> generate the required code to spawn these bridges.

CMakeLists.txt has a [custom target](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_bridge/-/blob/noetic/grpc/CMakeLists.txt#L6) to generate the code before the package is built.

Cog can also be manually run by first navigating to the package directory in a terminal and running the command Ex:
```
$ cd <workspace_folder>/src/phoenix_bridge
$ cog -r include/phoenix_bridge/write_conversions.hpp
```

### Scaling codegen for more message types

The main infrastructure to parse ROS2 message types and generate code is in place. However, when scaling up this bridge to parse more message types, there are some parts of the code that might need to be expanded.

1. Type casting - The protobuf types need to be cast into [IEC types](https://en.wikipedia.org/wiki/IEC_61131-3#Data_types), but more specifically, the types supported by PLCnext as seen in `ArpTypes.pb.h/CoreType` - [relative_link](include/phoenix_bridge/ServiceStubs/ArpTypes.pb.h#L81). This casting is handled in the [msg_parser](phoenix_bridge/src/parsers/msg_parser.py#L159) through the `TypesDict` dictionary. This dictionary has a few type castings already defined, but is not an exhaustive list. When new types are encountered which are undefined, the message `<type>_CASTING_UNDEFINED` can be seen in the generated code. To avoid/resolve this, `TypesDict` has to be extended by defining more casting pairs. The target is to cast from IDL type to PLCnext type.

2. Type name fixing - Some type names are different between the ROS msg types and protobuf type. Ex: ROS - `float64` = protobuf - `double`. These naming inconsistencies have to be fixed on an individual basis, and is done with the `adjust_type()` function in [msg_parser](phoenix_bridge/src/parsers/msg_parser.py#L30). Not only should this function be used in all cog scripts, the casting conditions also need to be updated as needed.

3. Type linking - In order to compile the package properly, when new ROS2 msg libraries are included, CMakeLists.txt should also be extended to include these so they can be linked. This is done by adding the type library to the variable [MSG_TYPES_TO_LINK](CMakeLists.txt#L26). Furthermore, `package.xml` should also be extended to add dependencies to the new libraries [here](package.xml#L20). This linking list is currently kept to the minimum used, in order to avoid linking every ROS type and bloating the package too much.

4. Special type handling - Some types need to be handled completely uniquely, and not covered by the standard templytes. Ex: The `time` type in ROS1 msgs is [specially converted to a double](include/phoenix_bridge/write_conversions.h#L164); arrays also need [special handling](include/phoenix_bridge/write_conversions.h#L153). These exceptional handling mechanisms are to be done directly in the cog scripts.

### Linting

Only the handwritten non-generated files of this project should be linted, and the generated files should be excluded. The include files are specified in the CMakeLists variable `files_to_lint`. Thereafter, standard [`roslint`](http://wiki.ros.org/roslint) is invoked for each of these files.

Additionally, install and run [`catkin_lint`](https://fkie.github.io/catkin_lint/) to ensure package definition is good.

### Testing

Tests can be manually run as follows:
1. Build the project: `catkin_make`
2. Build the tests: `catkin_make tests`
3. Run all tests: `catkin_make run_tets`
4. OR run individual tests: `rostest phoenix_bridge <test_name> --text`
5. See results: `catkin_test_results`

Tests shall also be run by CI automatically when code is pushed to gitlab.

### Unit Testing

Unit tests are added under the `tests/unit_tests` folder.

* `test_write_conversions`: Tests the conversion functions that convert from ROS msg type to grpc type.
* `test_read_conversions`: Tests the conversion functions that convert from grpc type to ROS msg type.

### Component Testing

Component testing is added under `tests/component_tests` folder.

* `test_topics`: Tests if the main bridge can be launched and if the publisher topics are created as specified.

## CI/CD

Consists of 3 phases:

1. Build and test the package using [industrial_ci](https://github.com/ros-industrial/industrial_ci). This runs all tests as configured in CMakeLists.txt

2. Doxygen documentation generation. The generated HTML documentation is available as downloadable artefacts and can also be served externally.

3. Dockerization builds docker images ONLY FOR TAGGED COMMITS. The commit tag is chosen as the tag for the image as well (make sure to update package.xml as well). Therefore, in order to trigger CD and build the images, tag the commit with the version of the package and push with tags. The created images are available in the [gitlab container repository.](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_bridge/container_registry/)


## How to build your specific PLCnext-ROS-Bridge as an APP
### Wiht CI Runner e.g. in GitLab
1. Fork this Repository
    More Information how to fork a repository you find in the [GitHub-Documentations](https://docs.github.com/de/get-started/quickstart/fork-a-repo).   

2. Modify depending on your demands.
    - The IDF file (`phoenix_bridge/config/interface_description.yaml`) with topics you want to publish/subscribe and the variablepaths which should be connected.
    - Add additonal ROS packages which should be availabe in the Image

3. Run the CI and collect the APP file.

4. Create a PLCnext Engineer Project
    
    Build a Application Projekt for the PLC with the Variables you want to connect to the ROS-Environment and run that on the Device.
    
    > NOTE: Don´t forget the global boolean Heartbeat Variable `Arp.Plc.Eclr/xLiveliness`! 

   [Getting started with PLCnext Engineer](https://www.plcnext.help/te/PLCnext_Engineer/Getting_started_with_PLCnext_Engineer.htm)

5. Install and start the APP via the WBM of the Controller
    > NOTE: Be sure that the Application with the variables which ar is running on the Device 

    More Information how to install a APP on the PLCnext Device you find in the [PLCnext Info Center](https://www.plcnext.help/te/WBM/WBM.htm).

6. **Enjoy the flexibility of PLCnext!**

### Manually without any CI Runner on Ubuntu OS
  1. Download the branch or the ROS distro you are interested in from this repository.
  2. Modify depending on your demands.
    - The IDF file (`phoenix_bridge/config/interface_description.yaml`) with topics you want to publish/subscribe and the variablepaths which should be connected.
    - Add additonal ROS packages which should be availabe in the Image

  3. Navigate to the downloaded directory for example in the Download folder.
      ```
      $ cd cd ~/Downloads/PLCnext-ROS-bridge
      ```
  4. Build the docker image for your ROS noetic.
      
      ```
      $ docker build --build-arg ROS_DISTRO=noetic --tag mybridge:noetic --file dockerfile_manually .
      ```
  5. Store the Image as .tar-file in the app directory
      
      ```
      $ cd ~/Downloads/PLCnext-ROS-bridge/app
      $ mkdir images
      $ docker save -o images/mybridge.tar mybridge:noetic
      ```
  6. Create a SquashFS container of the files for the APP.
      
      ```
      $ cd ~/Downloads/PLCnext-ROS-bridge/app
      $ sudo apt-get update
      $ sudo apt-get install --yes squashfs-tools rpm images
      $ docker inspect --format="{{.Id}}" mybridge:noetic | cut -d: -f2 >> ./image.id
      $ sed -i 's/[^:]*:\(.*\)/\1/' image.id
      $ sed -i "s/§§IMAGE_ID§§/$(<image.id)/g" app-compose.yml
      $ sed -i "s/§§IMAGE_ID§§/$(<image.id)/g" initscript.sh
      $ sed -i "s/§§TARGETS§§/AXC F 3152/g" app_info.json
      $ sed -i "s/§§ROS_BRIDGE_VERSION§§/2.0 specific/g" app_info.json
      $ sed -i "s/§§ROS_DISTRO§§/noetic/g" app_info.json
      $ chmod +x initscript.sh
      $ cd ..
      $ mksquashfs app plcnext-ros-bridge.app -force-uid 1001 -force-gid 1002
      ```
  7. Create a PLCnext Engineer Project
    
      Build a Application Projekt for the PLC with the Variables you want to connect to the ROS-Environment and run that on the Device.
    
      > NOTE: Don´t forget the global boolean Heartbeat Variable `Arp.Plc.Eclr/xLiveliness`! 

      [Getting started with PLCnext Engineer](https://www.plcnext.help/te/PLCnext_Engineer/Getting_started_with_PLCnext_Engineer.htm)

  8. Install and start the APP via the WBM of the Controller
      > NOTE: Be sure that the Application with the variables which ar is running on the Device 

      More Information how to install an APP on the PLCnext Device you find in the [PLCnext Info Center](https://www.plcnext.help/te/WBM/WBM.htm).
  9. **Enjoy the flexibility of PLCnext!**

