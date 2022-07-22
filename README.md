# PHOENIX BRIDGE

ROS package to bridge ROS Noetic to PLCnext devices. Requires the dependencies from the PLC SDK to be properly installed in order to compile and function. Refer [Phoenix Dependencies](https://gitlab.cc-asp.fraunhofer.de/ipa326/phoenix_dependencies/-/tree/noetic) repo for info on this.

The main principle is that this package can parse an interface description file and generate the code required for setting up bridging topics for the ROS msg types as specified.

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

TODO