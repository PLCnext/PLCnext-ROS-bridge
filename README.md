# PHOENIX BRIDGE

## Code generation with Cog

Using the following python3 package for code generation: [cog](https://nedbatchelder.com/code/cog/index.html#h_installation)

Read the types mentioned in the param file that the user wants to bridge -> generate the required code to spawn these bridges.

> NOTE: The codegen scripts currently only handle fixed length arrays inside ros messages, and skip variable length arrays

CMakeLists.txt has a [custom target](https://github.com/ipa-kut/phoenix_bridge/blob/f3cf41a185e6ff1fad2fad41b6b9e6fd9a187c0c/CMakeLists.txt#L6) to generate the code before the package is built.

Cog can also be manually run by first navigating to the package directory in a terminal and running the command Ex:
```
$ cd <workspace_folder>/src/phoenix_bridge
$ cog -r include/phoenix_bridge/write_conversions.hpp
```

## Developer Guide

Tests can be manually run as follows:
1. Build the project: `colcon build --symlink-install`
2. Run tests: `colcon test`
3. See results: `colcon test-result --verbose`

Tests shall also be run by CI automatically when code is pushed to gitlab.

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

1. `test_conversions.cpp`: Test the packWriteItem() and unpackReadItem() functions available in `include/phoenix_bridge/write_conversions.hpp`.   
@TODO: - [ ] Fill out the unit tests with meaningful code to manually unpack/pack the grpc_object respectively and test.


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
