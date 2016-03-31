#ihmc_ros

`ihmc_ros_core` provides the foundation for ROS integration with local and stable builds of IHMC Open Robotics Software. It includes our ROS messages, common configurations, and a dynamic Java bridge for launching arbitrary entry points in the IHMC software via roslaunch.

## Getting Started

The `ihmc_ros_core` project is meant to be included in a ROS catkin workspace. We currently support ROS Indigo on Ubuntu 14.04 LTS

### Packages

- `ihmc_msgs` contains the common custom IHMC ROS message files
- `ihmc_ros_common` contains common configurations and launches
- `ihmc_ros_java_adapter` provides a mechanism for integrating any executable entry point from the IHMC codebase with roslaunch via YAML configurations.

A good place to start is [the IHMC ROS Java Adapter](https://github.com/ihmcrobotics/ihmc_ros_core/tree/develop/ihmc_ros_java_adapter).

For more information, see the [GitHub Pages documentation](TODO fill this in).
