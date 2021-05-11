# Research Track 2 - assignment 1

#### Marco Gabriele Fedozzi [50833565]

## Content Description

The ROS2 rclcpp package here presented can dialogue with nodes written in a ROS1 package in order to control a non-holonomic mobile robot in a Gazebo environment. The user can choose to make the robot start moving or stop it: once the robot starts moving it will choose a random pose, drive towards it, and then repeat the process until being told to stop. Since the movement is here implemented as a service the user's inputs will be evaluated only once the robot reached its current goal, so it cannot be stopped "midway", but only in a goal pose.

In particular, the nodes contained in this package are:

- **position_service.cpp:** implements the server for the retrieval of a random (x,y,theta) pose;
- **state_machine.cpp:** takes care of calling for the random goal and sending it to the '/go_to_point' service when the user requires the robot to move.

## Compiling and Running

The package can be compiled as normal

```bash
path/to/ros2_ws/$ colcon build --packages-up-to rt2_assignment1
```

The two components can be run as individual nodes with

```bash
path/to/ros2_ws/$ ros2 run rt2_assignment1 comp_randomposserver
path/to/ros2_ws/$ ros2 run rt2_assignment1 comp_statemachine
```

However, a launch file instanciating a container and loading the components into it is provided for both efficiency and ease of use

```bash
path/to/ros2_ws/$ ros2 launch rt2_assignment1 sim_launch.py
```

---

In order to make this package communicate with the ROS1 residing "half" the third-party **ros1_bridge** package is used (see **Requirements** for more details), the mapping rules of which are expressed in the **mapping_rules.yaml** file. Hence, three steps are required to start the system
1. Launch the needed component from the ROS1 sided

```bash
path/to/ros_ws/src/rt2_assignment1$ roslaunch rt2_assignment1 sim_bridge.launch
```

2. Run the ROS1 bridge
```bash
path/to/ros2_ws/src/rt2_assignment1$ ros2 run ros1_bridge dynamic_bridge
```

3. Launch the container with the components implemented in this package
```bash
path/to/ros2_ws/src/rt2_assignment1$ ros2 launch rt2_assignment1 sim_launch.py
```

These three steps are pre-written in the bash script `bridge_launch.sh`, so that the whole process boills down to
```bash
path/to/ros2_ws/src/rt2_assignment1$ ./bridge_launch.sh
```


## Implementation Details

### StateMachine

Notice that, being this a component, no spinning is done inside the class. Instead, every service call is implemented asynchronously, with response-related operations done in a dedicated callback. To retrieve a new goal pose once the previous one has been reached a recursive behaviour _could've been_ implemented, but being the '/go_to_point' callback treated as blocking by the ROS1 bridge (despite it being asyn in ROS2), this could've resulted in an endless blocking service call (imagine a scenario in which the callback to '/user_interface' calls for the '/go_to_point' service directly, which recursively launches a new call once the previous one returned, never passing control back to the original callback).
To avoid this, and in general make the code more robust and readable, a periodic timer has been implemented, with a function binded to its expiration: inside this callback it's checked whether the robot has reached its previous goal position and if the user didn't ask for it to be stopped: if both these conditions hold then the go_to_point-related callback is issued, asking for a new target pose and making the robot move towards it by calling the '/go_to_point' service.


## Documentation

Beside this README further documentation of all classes and methods can be found in the **doc** folder.


## Requirements

This package must be run together with a compliant ROS1 package containing the definition of the service here used, as well as nodes implementing a '/go_to_point' service (in this particular case implemented in the **main** branch).

Furthermore, the `ros1_bridge` package must be installed and compiled (see ![ros1_bridge](https://github.com/ros2/ros1_bridge) for more details).

In order to launch the `bridge_launch.sh` bash script gnome-terminal must be installed in the system
```bash
sudo apt install gnome-terminal
```

## Known Issues and Limitations

Despite the service calls being implemented as asynchronous in the ROS2 components, they are translated back into blocking ones when interfaced via ros1_bridge.
Furthermore, the implementation of that node has an intrinsic timeout for service calls which, combined to the extensive calls here used to drive the robot,
can lead to reported failures in calling '/user_interface' service.
To patch that the timeout can be increased by modifying its value in the **ros1_bridge** source code,
namely at line 314 of file **include/factory.hpp**; 60 seconds should be enough.

Notice however that this workaround seems to not be enough when calling the nodes from the provided `bridge_launch.sh`,
since the error can still arise sporadically there. Nevertheless, while error messages pop up in the terminal the system itself
does not esperience any problem, since the services do terminate correctly and the exceptions are caught in the scripts.
