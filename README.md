# PHOENIX BRIDGE

## Code generation with Cog

Using the following python3 package for code generation: [cog](https://nedbatchelder.com/code/cog/index.html#h_installation)

Read the types mentioned in the param file that the user wants to bridge -> generate the required code to spawn these bridges.   

CMakeLists.txt has a [custom target](https://github.com/ipa-kut/phoenix_bridge/blob/f3cf41a185e6ff1fad2fad41b6b9e6fd9a187c0c/CMakeLists.txt#L6) to generate the code before the package is built.

Cog can also be manually run by first navigating to the package directory in a terminal and running the command:
```
$ cd <workspace_folder>/src/phoenix_bridge
$ cog -r include/phoenix_bridge/conversions.hpp
``` 
