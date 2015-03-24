#ihmc_ros

`ihmc_ros` provides ROS integration with and a stable binary software distribution of our Whole Body Controller for humanoid robots.

## Getting Started

The `ihmc_ros` project is meant to be included in a ROS catkin workspace and requires ROS Indigo.

### Packages

- `ihmc_msgs` contains all of the custom IHMC ROS message files
- `ihmc_models` provides robot models for use with the robot description ROS topics and RViz visualization
- `ihmc_sim` provides a stable binary distribution of our control software integrated in to Simulation Construction Set, a simulation environment for developing walking algorithms.
- `ihmc_diagnostics` provides a collection of tools/scripts for trouble-shooting the Java installation and the behavior of the controller.
- `ihmc_atlas` provides integration with the Boston Dynamics Atlas robot outside of simulation. Atlas teams that have the appropriate clearance from Boston Dynamics can find more information at [IHMC Atlas Controller Releases](https://bitbucket.org/ihmcrobotics/ihmc-atlas-controller-releases)

For more information, see the [wiki here on BitBucket](https://bitbucket.org/ihmcrobotics/ihmc_ros/wiki) and the README.md files in each package.
