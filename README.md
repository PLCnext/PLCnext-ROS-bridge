# PHOENIX BRIDGE

Playing around with architecture for now. Serves as a base for starting discussion with client.   
Very very open to critique.     
Read code comments. Trace emtry point from the launch file.         
Once the architecture is semi finalized with client, will move this to gitlab.   

## Code generation with Cog

Using the following python3 package on noetic for code generation: [cog](https://nedbatchelder.com/code/cog/index.html#h_installation)

Read the types mentioned in the param file that the user wants to bridge -> generate the required code to spawn these bridges.   

Template based generation for the following files:   
* `include/phoenix_bridge/include_types.h`
*  `include/phoenix_bridge/phoenix_bridge.h`
* `src/phoenix_bridge/include_types.cpp`

The templates are written into these files. CMakeLists.txt has a [custom target](https://github.com/ipa-kut/phoenix_bridge/blob/f3cf41a185e6ff1fad2fad41b6b9e6fd9a187c0c/CMakeLists.txt#L6) to generate the code before the package is built.

PROS: Can generate lean code based on what the user wants. Has the potential to allow the user to add custom msg types without any code changes. (just add the new msgs under the msg folder and include it in cmake -> mention in param file -> build)   
CONS: Bridge HAS to be built from source, cannot release binaries.

Discuss with client what they want and go with that.
