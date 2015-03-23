# ihmc_msgs

`ihmc_messages` provides a collection of ROS messages and a software distribution that responds to these messages for interacting with the IHMC Whole Body Controller.

## Getting Started

The `ihmc_msgs` project is meant to be included in a ROS catkin workspace.

To use `ihmc_msgs`, check the project out in to the `src` directory of your catkin workspace, perform a `catkin_make`, and source the generated setup file for your shell.  If you're upgrading from a previous version of `ihmc_msgs`, you should `rm -rf` the `build` and `devel` directories in your catkin workspace before running `catkin_make`

You should then `rosrun ihmc_msgs ihmcBootstrap.py` to download the binary distribution of the IHMC ROS API.

The following roslaunch files can be launched in the package:
- `ihmc_robot.launch` for starting the API node that will talk to the real controller.
- `ihmc_sim.launch` for launching a simulator and its API node
- `display_simulation.launch` which will bring up an rviz visualization (currently only for the sim node)
- `ihmc_sim_diagnostic.launch` will make the robot go through a series of movements in simulation

`ihmc_msgs` requires a small amount of network configuration, including additional specification of the relevant ROS API's as ROSJava doesn't automatically inherit its ROS information from the shell environment the way native ROS does. This is done using the IHMCNetworkParameters.ini file.

## IHMCNetworkParameters File

In `ihmc_msgs/scripts/<API distribution>/bin` is the IHMCNetworkParameters.ini file. The fields in this file default to localhost, but they can be modified if you are running in a more distributed manner.

- `rosURI` corresponds to the ROS\_MASTER\_URI environment variable, without the _http://_ directive. You can change the hostname and port to match what you use as your ROS\_MASTER\_URI here. Shell expansion is not available in this file so you will have to type the hostname.
- `robotController` corresponds to the host/IP of the computer running the IHMC Whole-Body Control algorithm.  In Sim, this is localhost. If you're using `ihmc_msgs` with a real robot, this may not be the case.
- `networkManager` is the host/IP of the computer running the API node. Localhost is an acceptible entry here, unless you are running on a machine with multiple hardware interfaces and you would like to force binding to a particular interface.

If you don't want to use the default location for the .ini file, you can use the roslaunch arg "ihmc\_network\_file" to specify a different file, e.g. `roslaunch ihmc_msgs ihmc_sim.launch ihmc_network_file:=$HOME/OurTeamNetworkFile.ini`

## Box Step demo

To verify that the setup works without having to hand craft any ROS Messages, you can `rosrun ihmc_msgs boxStep.py` to run the box step script, which will automatically pipe ROS messages in to the API. The unmodified box step script should step in place a four times, and then walk forward, left, back, and right in approx. a 0.4m square box.

## Simulation Parameters
The following parameters can be set in the `ihmc_sim.launch` file.

### Starting Position
The starting position in the demo world can be defined. Currently the options are:
  DEFAULT, DRC_TRIALS_TRAINING_WALKING, DRC_TRIALS_QUALS, MID_LADDER, RAMP_BOTTOM, RAMP_TOP, NARROW_DOORWAY, BARRIERS, SMALL_PLATFORM, MEDIUM_PLATFORM, ON_MEDIUM_PLATFORM, EASY_STEPPING_STONES, STEPPING_STONES, STAIRS, ROCKS, LADDER, IN_FRONT_OF_ZIGZAG_BLOCKS, SINGLE_CYLINDERBLOCKS, TOP_OF_SLOPES, DEFAULT_BUT_ALMOST_PI, IN_FRONT_OF_CINDERBLOCK_FIELD, IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS, IN_FRONT_OF_CYLINDER_BLOCKS, IN_FRONT_OF_SLANTED_CINDERBLOCK_FIELD, OFFSET_ONE_METER_X_AND_Y, OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, SMALL_PLATFORM_TURNED, SMALL_WALL