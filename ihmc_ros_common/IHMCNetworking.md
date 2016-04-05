# IHMCNetworkParameters File

The various components of the IHMC Robotics software architecture require a networking configuration for different pieces to talk to each other. From a high level overview, a typical IHMC setup has three primary components, and for the sake of simplicity all of these processes/components should use the same configuration (with a possible slight caveat on the logger configuration, more on that later). The components are:

1. The robot control algorithm
2. The IHMC Network Processor; this is a non-realtime node used for things high-level behaviors, the networked API endpoints, and vision. ROS messages are received here and translated in to commands for the controller
3. A "logger". This component may or may not exist, but the IP address associated with the logger is actually a specification of an endpoint for a protocol used by many other tools, so this configuration is still often times important.

## Specifying configuration using environment variables

There are 4 variables that all IHMC processes need to be aware of. They are:

1. `ROS_MASTER_URI`: this is pulled from your system's environment variables, just like any other ROS processes
2. `IHMC_LOGGER_IP`: This is the IP address used for the "logger" protocol, which is actually used by various other tools as well. More on this later.
3. `IHMC_ROBOT_CONTROLLER_IP`: This is the IP address of the machine that is running the control algorithm for the robot/simulation
4. `IHMC_NETWORK_MANAGER_IP`: This is the IP address of the machine that is running the IHMC Network Processor/ROS API

There is an additional, optional variable that can be set on the same machine as the `IHMC_ROBOT_CONTROLLER_IP`, that is specific to the IHMC Logger:

- `IHMC_LOGGED_CAMERAS`: This is a comma-separated list of integers specifying which cameras (indexes as reported by the Blackmagic SDK) are associated with that robot for logging purposes.

## Specifying configuration using a configuration file

In the `configurations` directory of many packages you will find the IHMCNetworkParameters.ini file (or some variant there of). The fields in this file default to localhost. The fields have names that correspond to the environment variables listed above.

**NOTE that IHMC Configuration Files will *always* be overridden by environment variables.**

The .ini file is basically a key-value association of strings -> strings, and it uses the following syntax:

```ini
key=value
```

In .ini files, colons must be escaped, so when specifying the ROS Master URI, you'll want to escape the colon in the protocol identifier (e.g. http) and the port identifier.  The ini keys that correlate to the environment variables are:

1. `ROS_MASTER_URI` -> `rosURI`. Once again, note that if your system has `ROS_MASTER_URI` exported that it will overrite a URI specified in this files
2. `IHMC_LOGGER_IP` -> `logger`
3. `IHMC_ROBOT_CONTROLLER_IP` -> `robotController`
4. `IHMC_NETWORK_MANAGER_IP` -> `networkManager`
5. `IHMC_LOGGED_CAMERAS` -> `loggedCameras`

A simple Network Parameters template file is available in this repository. It looks like this:

```ini
rosURI=http\://localhost\:11311
logger=localhost
robotController=localhost
networkManager=localhost

#don't log any cameras
loggedCameras=
```

## About the Logger IP and Logging Protocol

As mentioned above, the `logger` key is used for more than just logging. Additionally, due to the nature of the logging protocol, the usage of IP addresses is done a bit differently than one would expect intuitively. The `logger` key should be an IP address but it does *not* need to point specifically at a machine running the IHMC Logger. First we'll explain what the protocol is for, then we'll explain what the IP Address is *actually* pointing at.

The protocol uses a server-client architecture with an announcement/handshake frontend. **Servers** are things that stream data out, and **Clients** are things that consume the data and do something in it. For example, a robot controller may implement a Server to stream its state data, and various Clients can connect to do useful things with this data; an SCS based remote visualizer can connect to plot data and visualize the robot, an IHMC Logger instance can connect to log robot runs. The protocol was originally develop was specifically for logging, hence the legacy name of `logger` for the key, but it has grown to accommodate much more.

The logger infrastructure is set up in such a way that we expect the various machines in the network can and will have multiple network interfaces, potentially on completely separate subnets, with possibly no routing between the subnets. While complicated, this is common in many non-trivial robotics labs and robotics field setups so we have considered this in the configuration. With this information, we discuss the logger IP usage:

- **On the machine running the server (e.g. the robot controller)**: The `logger` key should point to an IP address that is on the subnet that the server will be streaming its data out over. If you have an IHMC Logger, you'll point to an IP on the same subnet that the logging machine is on. Often times, the easiest IP address to use is the IP address of the IHMC Logger instance itself. Hence the legacy name of this key.

- **On the machine running the client**: The `logger` key should point to an IP address that is on the subnet that can see the server you wish to connect to. If the client is an IHMC Logger instance, you can easily use the machine's own IP address, ***BUT it is important that you do not use `localhost` on a machine with multiple interfaces***. You can absolutely have the machine point at itself as long as you specify the IP address that is on the interface that can see the server. If the client is something like a remote visualizer, you can usually follow the same guidance or if you have an IHMC Logger instance on your network then you can point at that as well to unify your configuration.
