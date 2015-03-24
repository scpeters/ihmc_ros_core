#ihmc_sim

##Usage

`ihmc_atlas` provides the `ihmc_atlas.launch` file, which starts up the IHMC ROS API configured to talk to the real robot.

You can set the following roslaunch args:

- `ihmc_network_file:=<absolute path to network file>`: Specific the network configuration .ini file for the IHMC software. See [the wiki](https://bitbucket.org/ihmcrobotics/ihmc_ros/wiki/network-config) for more information
- `ihmc_model:=<MODEL_NAME>`: Specify an argument for the IHMC Controller letting it know which model to use internally. If you would like to see additional models, please feel free to submit a [feature request](https://bitbucket.org/ihmcrobotics/ihmc_ros/issues/new). Currently valid model arguments:
  - ATLAS_UNPLUGGED_V5_NO_HANDS
  - ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ
- `description_model:=<path to urdf file>`: The .urdf used by ROS when publishing robot descriptions. Defaults to one of the models vendored in the `ihmc_models` package, but can be overriden.
- `starting_location:=<STARTING POSITION>`: Specify landmarks in the test environment to spawn the robot near.

### Starting Positions
The starting position in the demo world can be defined. Currently the options are:

    DEFAULT, DRC_TRIALS_TRAINING_WALKING, DRC_TRIALS_QUALS, MID_LADDER, RAMP_BOTTOM, RAMP_TOP, NARROW_DOORWAY, BARRIERS, SMALL_PLATFORM, MEDIUM_PLATFORM,   ON_MEDIUM_PLATFORM, EASY_STEPPING_STONES, STEPPING_STONES, STAIRS, ROCKS, LADDER, IN_FRONT_OF_ZIGZAG_BLOCKS, SINGLE_CYLINDERBLOCKS, TOP_OF_SLOPES,   DEFAULT_BUT_ALMOST_PI, IN_FRONT_OF_CINDERBLOCK_FIELD, IN_FRONT_OF_TWO_HIGH_CINDERBLOCKS, IN_FRONT_OF_CYLINDER_BLOCKS, IN_FRONT_OF_SLANTED_CINDERBLOCK_FIELD,   OFFSET_ONE_METER_X_AND_Y, OFFSET_ONE_METER_X_AND_Y_ROTATED_PI, SMALL_PLATFORM_TURNED, SMALL_WALL
