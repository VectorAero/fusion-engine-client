# ROS2 Package for NavPointOne Atlas GPS system

There are two Packages, atlas_gps and atlas_msgs
Atlas_gps is the package ROS2 package which interfaces with the Atlas.
Atlas_msgs contain definitions and build instructions for the ROS2 messages GPSFix and GPSStatus.

You will need to build both of the packages using colcon. Copy the atlas_gps and atlas_msgs directories to the ROS2 workspace src directory.

You will then need to build the packages. For example:

<blockquote>
$ colcon build --packages-select atlas_gps</blockquote>

You may choose to build the packages one at a time, or selectively as above.

In order to run the package:

<blockquote>
$ . install/setup.bash
$ ros2 run atlas_gps atlas</blockquote>

This will start the ROS2 package, converting the Atlas messages to ROS2 messages and publishing them.

In the settings page for the Atlas, you will need to set the appropriate Output messages:

* ROS Pose
* ROS GPS Fix
* ROS IMU

The ROS2 node is listening for these messages specifically. You can exclude any of these if desired.

In setting up the Atlas to use the package, the 'FusionEngine Client binary interface:' should be set to 'UDP Client' with the IP address set to that of the Jetson AGX Xavier, and the port default set to 12345.


