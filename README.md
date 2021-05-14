# Research Track 2 - assignment 1

#### Marco Gabriele Fedozzi [50833565]

```
rt2_assignment1/
  |
  launch/         - launch files
    |
    sim.launch            - Gazebo simulation
    sim_bridge.launch     - python scripts only launch
    sim_coppelia.launch   - nodes only launch
  scripts/        - python scripts
    |
    go_to_point.py        - pyhton script controlling the robot
    user_interface.py     - minimal command line UI
  src/            - C++ source code
    |
    position_service.cpp  - returns random position
    state_machine.cpp     - manages the FSM logic
  srv/            - custom services
    !
    Command.srv           - user UI input
    Position.srv          - goal position
    RandomPosition.srv    - bound random pose
  urdf/           - robot description for Gazebo simulation
    |
    my_robot.urdf         - mobile robot description
  cplScenes/      - CoppeliaSim scenes
    |
    pioneer_scene.ttt     - Pioneer p3dx scene
    robotnik_scene.ttt    - Robotnik Summit XL140701 scene
  CMakeLists.txt  - CMake file
  README.md       - this very file
  package.xml     - manifest
```

## Package Description

This package controls a mobile non-holonomic robot with a simple 'go_to_point' behaviour:
1. a random goal is issued (a _pose_, [x,y,theta]);
2. the robot orients itself towards the [x,y] destination;
3. it then drives straight to that position (adjusting the orientation if need be);
4. having reached the [x,y] goal position the robot turns in place in order to match the goal _theta_;
5. if the user does not stop the robot GOTO step 1, otherwise stay still until asked to start again, then GOTO step 1;

Since the user request is here implemented as a service it cannot be preempted, stopping the robot is possible only once a goal is reached, restarting it when issuing a new goal.

## Contents Explanation

This package contains 4 nodes managing the aforementioned control of a mobile robot.
Two nodes are implemented as python scripts
- **go_to_point.py**: the service server managing the robot speed control depending on the goal received.
- **user_interface.py**:  the simple command line user interface, which sends the requests to start/stop the go_to_point behaviour.

Whilst the last two are C++ based nodes
- **position_service.cpp**: the server generating a random pose [x,y,theta] as a response to a request.
- **state_machine.cpp**:  the FSM managing the request of a new pose when needed, sending it as a goal request to 'go_to_point' service server.

---

Finally, the control can be applied to a robot simulated using Coppeliasim (see **Requirements**), for which two scenes are here presented
- **pioneer_scene.ttt**: a simple scene with a Pioneer p3dx non-holonomic mobile robot in an empty environment.
- **robotnik_scene.ttt**: a simple scene with a Robotnik Summit XL140701 non-holonomic mobile robot in an empty environment.

## Compiling and Running

Compilation can be carried out as always with
```bash
path/to/ros_ws/$ catkin_make
```

Three launch files are provided:
- **sim.launch**: to be used in order to launch all the nodes and the Gazebo simulation.
```bash
path/to/ros_ws/$ roslaunch rt2_assignment1 sim.launch
```
In this case the Gazebo simulation will automatically start.

- **sim_coppelia.launch**: to be used in order to launch all the nodes which will communicate with the Coppelia simulation.
```bash
path/to/ros_ws/$ roslaunch rt2_assignment1 sim_coppelia.launch
```
In this case CoppeliaSim must be started separately (remember to have an instance of roscore running before launching the CoppeliaSim executable). The simulation can be either started before or after launching the nodes, but do not try to run a new simulation when the nodes have already been running on a previous instance or the system could find itself in an initial state different from the assumed one, never being able to reach the goal. In other terms, each time the simulation is restarted the nodes should be restarted to, and vice-versa (generally).

- **sim_bridge.launch**: to be used in order to launch only the Python scripts, leaving the rest to the components implemented in the ROS2 package, to which this is bridged using 'ros1_bridge' (see the README on the _'ros2'_ branch for further info)
```bash
path/to/ros_ws/$ roslaunch rt2_assignment1 sim_bridge.launch
```

## Documentation

No real documentation is provided here, since the nodes were already written and their behaviour is pretty similar to what is presented in the branch _'action'_, which is instead extensively documented.

## Requirements

**Gazebo** is required to run the first launch file (the scene definition is presented in this package).
**Coppeliasim** is required to run the second launch file [download link](http://www.coppeliarobotics.com/downloads.html)
**ros1_bridge** ROS2 package, as well as the **ros2** branch of this repository cloned in a ROS2 workspace in order to run the third launch file (again, it's _strongly_ suggested to read the README on that branch for further information).

## Known Issues and Limitations

If you try running both the Gazebo and CoppeliaSim and the latter seems to not respond to the nodes, whilst the user interface results frozen after having told the system to run, try to **kill the roscore process**; this might be related to Gazebo overwriting some values related to the simulation (probably simulation time) and these not being appropriately "cleaned" once Gazebo is closed.
