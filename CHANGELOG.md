## 0.9.1

Fix a bug in the ROS Java Adapter that prevented it from downloading .jars correctly.

Some context can be found here: https://github.com/bytedeco/javacv/issues/395

Basically we already applied the above fix to our build system that emits the .jars but we didn't add it to the gradle scaffolding in this project that also downloads the .jars.

## 0.9.0

Release 0.9.0 introduces a semi-long-term support version of the ROS API. 
The .msg files in `ihmc_msgs` here should remain stable for the foreseeable future unless a change is needed by the userbase.

The primary changes are the introduction of the AdjustableFootstepMessage for replanning a step mid-swing, and the introduction
of additional timing elements in Footstep messages and Footstep List messages. Consult the comments in the .msg files for more information.