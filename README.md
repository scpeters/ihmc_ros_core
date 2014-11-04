# ihmc_msgs

`ihmc_messages` provides a collection of ROS messages and a software distribution that responds to these messages for interacting with the IHMC Whole Body Controller.

## Getting Started

If using `ihmc_msgs` with the real Atlas robot, the computer needs to have at least one network interface that is on the Boston Dynamics Robot's subnet and that has the full IP address of 10.66.171.44. If this will collide with existing software, you can change this configuration once the distribution has been downloaded (see below) by modifying the `networkProcessorIP` value in `scripts/<distribution>/bin/Configurations/atlas_network_config.ini`

The `ihmc_msgs` project is meant to be included in a ROS catkin workspace.

To use `ihmc_msgs`, check the project out in to the `src` directory of your catkin workspace, perform a `catkin_make`, and source the generated setup file for your shell.  If you're upgrading from a previous version of `ihmc_msgs`, you should `rm -rf` the `build` and `devel` directories in your catkin workspace before running `catkin_make`

You should then `rosrun ihmc_msgs ihmcBootstrap.py` to download the binary distribution of the IHMC ROS API.

There are three roslaunch files in the package: `ihmc_robot.launch` for starting the API node that will talk to the real controller, `ihmc_sim.launch` for launching a simulator and its API node, and `display_simulation.launch` which will bring up an rviz visualization (currently only for the sim node).

If the `IHMCAtlasAPI` is the only ROS node active, it will handle setting up its own ROS core and handling IP address resolution.  If you will be running additional ROS tools and have your own ROS Master URI set for this purpose, you will need to modify the appropriate parameter in `scripts/<distribution>/bin/Configurations/atlas_network_config.ini`.

## Box Step demo

To verify that the setup works without having to hand craft any ROS Messages, you can `rosrun ihmc_msgs boxStep.py` to run the box step script, which will automatically pipe ROS messages in to the API.  The unmodified box step script should step in place a few times, and then walk forward, left, back, and right in approx. a 1m square box.