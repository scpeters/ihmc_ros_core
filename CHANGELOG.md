## 0.9.2

Hotfix for a regression introduced in November in this commit: https://github.com/ihmcrobotics/ihmc_ros_core/commit/8e971958ba29c9659fc39b869b6105ddcea81058#diff-4b2e14f576a4443fe5693bbf8a2a1217

While it fixed the adapter for source builds, it can actually cause problems for non-source builds. This hotfix release makes modifications to the adapter so that it will completely avoid the warmUp task unless it is explicitly requested as an argument to a gradle command (which the warmup launch scripts do).

## 0.9.2

Fix a bug in the ROS Java Adapter that prevented it from downloading .jars correctly.

Some context can be found here: https://github.com/bytedeco/javacv/issues/395

Basically we already applied the above fix to our build system that emits the .jars but we didn't add it to the gradle scaffolding in this project that also downloads the .jars.

## 0.9.0

Release 0.9.0 introduces a semi-long-term support version of the ROS API. 
The .msg files in `ihmc_msgs` here should remain stable for the foreseeable future unless a change is needed by the userbase.

The primary changes are the introduction of the AdjustableFootstepMessage for replanning a step mid-swing, and the introduction
of additional timing elements in Footstep messages and Footstep List messages. Consult the comments in the .msg files for more information.