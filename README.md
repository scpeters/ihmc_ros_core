# ihmc_msgs

`ihmc_messages` provides a collection of ROS messages and a software distribution that responds to these messages for interacting with the IHMC Whole Body Controller.

## Getting Started

The `ihmc_msgs` project is meant to be included in a ROS catkin workspace. 

To use `ihmc_msgs`, check the project out in to the `src` directory of your catkin workspace, perform a `catkin_make`, and source the generated setup file for your shell.

You should then `rosrun ihmc_msgs ihmcBootstrap.py` to download the binary distribution of the IHMC ROS API.

There are three roslaunch files in the package: `ihmc_robot` for starting the API node that will talk to the real controller, `ihmc_sim` for launching a simulator and its API node, and `display_simulation` which will bring up an rviz visualization (currently only for the sim node).
