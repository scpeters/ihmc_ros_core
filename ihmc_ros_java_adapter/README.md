#ihmc\_ros\_java\_adapter

The IHMC ROS Java Adapter is a utility for launching arbitrary Java applications (main methods) without needing to interact with Java or an IDE directly. In the ROS context, this adapter is used to allow for launching IHMC Java applications via `roslaunch` without the underlying Java code or build systems needing to be ROS or Catkin aware. It is powered by the [Gradle build system](https://gradle.org), and utilizes an included [Gradle wrapper](https://docs.gradle.org/current/userguide/gradle_wrapper.html). Knowledge of Gradle is not required to use the adapter.

To use the adapter, a configuration must be expressed by creating a .yaml file that lets the adapter know some information about the Java program you would like to launch, and then the configuration must be passed to a Gradle task. `roslaunch` is just one way of forwarding this configuration to the Gradle task, and is the one that we will discuss most often.

## Usage

### The .yaml file

The yaml configuration for the adapter is fairly small. A sample yaml looks like this:

```yaml
!ihmc_ros_java_adapter
programArgs: []
vmConfig:
  heapSize: 4g
  mainMethod: us.ihmc.valkyrie.ValkyrieROSAPISimulator
dependencies:
- us.ihmc:Valkyrie:0.9.0
```

This is the yaml file used to launch the Valkyrie SCS simulation in the [ihmc_valkyrie_ros](https://github.com/ihmcrobotics/ihmc_valkyrie_ros) package. The custom tag `!ihmc_ros_java_adapter` tells the yaml parser what to expect in this configuration file. The other entries are as follows:

- `programArgs`: The arguments that should be passed *to the Java application*. These are not arguments to the Gradle task, the JVM, or to ROS.
- `vmConfig`: Configurations for the Java VM that will be used to run the specified Java application. Currently this only supports two entries, more flexibility will be introduced as required:
    - `heapSize`: A string that indicates how much heap size should be provided to the JVM. It is the same as the Java `-Xmx` argument and uses the same syntax, a numeric value in bytes. You can use `k` or `K`, `m` or `M`, and `g` or `G` as convenience for kilobytes, megabytes, and gigabytes. This example allocates 4 gigabytes of RAM for the JVM. Note the lack of a space between the number and the byte prefix.
    - `mainMethod`: The fully qualified class name of the class containing the `main(String[] args)` method you would like to run.
- `dependencies`: A YAML Sequence of Maven artifacts specified in `group:artifact:version` format. For many applications there could very easily only be one of these. IHMC Maven artifacts can be found on [our Bintray](https://bintray.com/ihmcrobotics/maven-release). In this case, we want to use our Valkyrie distribtion, so we can find the [Valkyrie package on Bintray](https://bintray.com/ihmcrobotics/maven-release/Valkyrie), click on the version we wish to use, and we Bintray will show us the `groupId`, `artifactId`, and `version` of the resource that we can list as a dependency:
![Bintray](http://d.pr/i/vMUY+)

### Using the adapter from roslaunch

The simplest requirement for launching a Java application via roslaunch is to specifiy a node that invokes the Gradle wrapper with the appropriate task and telling it the name of the Yaml file. In its simplest form:

```xml
<launch>
  <node name="MyNode" pkg="ihmc_ros_java_adapter" type="gradlew" args="runJava -Pyaml=/path/to/my/yaml/file.yaml" required="true" output="screen" cwd="node" />
</launch>
```

The things to note are the `pkg`, `type`, and `args` attributes. In typical `roslaunch` fashion, `pkg` specifies the package in which the node's executable lives, and `type` is the name of the executable to be run by the node (in this case, `gradlew` is the Gradle wrapper script). The minimum required `args` are "runJava" which is the special Gradle task, and -Pyaml=[...]. The `-P` syntax is Gradle syntax, and is the mechanism for passing command line arguments to the Gradle script itself.

### Advanced Usage

#### Using locally built versions of IHMC Maven artifacts

If you have [IHMC Open Robotics Software](https://github.com/ihmcrobotics/ihmc-open-robotics-software) cloned on your machine, you can "override" any .yaml that declares an **explicit** dependency on an IHMC Maven artifact that originates from IHMC Open Robotics Software. This currently does not support the standalone smaller libraries like ihmc_ros_control or any other library not contained in Open Robotics Software. In order to do this, you need to do two things:

- The environment variable `IHMC_SOURCE_LOCATION` must be set, and should point to the clone of the software. For example:
```bash
$ cd ~
$ git clone git@github.com:ihmcrobotics/ihmc-open-robotics-software.git
$ export IHMC_SOURCE_LOCATION=$HOME/ihmc-open-robotics-software
```

- You must pass an argument to the Gradle script: `-PuseLocal=true`.

An example of how to pass this argument to Gradle via roslaunch could look something like this:

```xml
<launch>
  <arg name="use_local_build" default="false" />
  <node name="MyNode" pkg="ihmc_ros_java_adapter" type="gradlew" args="runJava -Pyaml=/path/to/my/yaml/file.yaml -PuseLocal=$(arg use_local_build)" required="true" output="screen" cwd="node" />
</launch>
```

Here, we expose a roslaunch argument and forward it to the Gradle script so that you can switch between stable release and local build using a command line argument. You could also hard-code the parameter in differing launch scripts for "stable" and "development" if you'd prefer.

#### Overriding program arguments

You can also specify the program arguments for the Java application by passing them in to the Gradle script if you want. **Note that if you use this, the Adapter will ignore all program arguments specified in the .yaml**.

This is done by specifying `-PprogramArgs=` and then a comma delimited, **no white space** list of arguments/flags you want to pass to the Java program.

#### Setting IHMC Java System Properties

Java has a notion of System properties, command line arguments delimited by a `-DpropertyName` syntax. To forward system properties from the Gradle adapter to the running application, they must be in the "us.ihmc" namespace in the Java code. Currently, there is only one IHMC system property exposed in this way, which is the system property that locates our Network Parameter file. Inserting a `-Dus.ihmc.networkParameterFile=` as a command line argument to the Gradle wrapper (e.g. in the `args` of the node in `roslaunch`) will forward this to the Java program. Hopefully this will eventually go away and we will allow for specifying System Properties in the YAML files.

For examples of this advanced usage as well as the others mentioned above, take a look at [this launch script from ihmc_valkyrie_ros](https://github.com/ihmcrobotics/ihmc_valkyrie_ros/blob/develop/launch/ihmc_valkyrie_scs.launch):

```xml
<launch>
  <param name="use_sim_time" value="true"/>
  <arg name="use_local_build" default="false" />
  <arg name="ihmc_network_file" default="$(find ihmc_valkyrie_ros)/configurations/IHMCNetworkParametersSim.ini" />
  <arg name="description_model" default="$(find val_description)/model/urdf/valkyrie_sim.urdf" />
  <arg name="starting_location" default="DEFAULT" />
  <arg name="tf_prefix" default="NONE" />
  <arg name="scs_args" default="" />

  <include file="$(find ihmc_ros_common)/launch/robot_description_common.launch">
    <arg name="robot_name" value="valkyrie" />
    <arg name="model_file" value="$(arg description_model)" />
  </include>

  <include file="$(find ihmc_valkyrie_ros)/launch/common/ihmc_valkyrie_params_common.launch" />

  <node name="IHMCValkyrieROSAPISimulator" pkg="ihmc_ros_java_adapter" type="gradlew" args="runJava -Dus.ihmc.networkParameterFile=$(arg ihmc_network_file) -PuseLocal=$(arg use_local_build) -Pyaml=$(find ihmc_valkyrie_ros)/configurations/scs.yaml -PprogramArgs=-s,$(arg starting_location),--tfPrefix,$(arg tf_prefix),$(arg scs_args)" required="true" output="screen" cwd="node">
  </node>
</launch>
```
