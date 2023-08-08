# pacmod4_common

This repo contains code for both parsing and encoding pacmod4 CAN messages. 
The code related to parsing and encoding CAN frames is auto-generated thanks to the helpful [c-coderdbc](https://github.com/astand/c-coderdbc) tool.
The rest of the code is manually written and is used for packing and unpacking ROS [pacmod4_msgs](https://github.com/astuff/pacmod4_msgs).
The pacmod4_common library can be compiled under both ROS1 and ROS2 in order to reduce duplication and code maintenance efforts.

## Version Support

Below is a table of supported DBC versions.

|Major Version|Full DBC Version|
|-|-|
|13|13.2.0|
