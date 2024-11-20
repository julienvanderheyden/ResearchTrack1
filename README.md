# Assignment 1: Turtlesim Control and Distance Monitoring

This package, **`assignment1_rt`**, contains two nodes designed to enhance the functionality of the **Turtlesim** environment. It provides control over multiple turtles and ensures collision-free operation within the simulation.

---

## Overview

### 1. **UI Node**
The **UI** node spawns a second turtle in the Turtlesim environment and allows the user to control both turtles.

- **Input Format**: The user enters commands directly in the terminal, using the format:

```php
<turtle_number> <linear_velocity> <angular_velocity>
```

+ `turtle_number`: Selects the turtle to control (`1` for the default turtle or `2` for the newly spawned turtle).
+ `linear_velocity`: The forward/backward speed of the turtle (clamped between `-2.5` and `2.5`).
+ `angular_velocity`: The rotational speed of the turtle (clamped between `-1.5` and `1.5`).

- **Behavior**:
+ Velocities exceeding the thresholds are automatically clamped.
+ The entered command is applied for **1 second**, after which the user can input a new command.

---

### 2. **Distance Node**
The **Distance** node monitors the relative distance between the turtles and ensures safe operation.

- **Published Topic**:
+ `/relative_distance` (`std_msgs/Float32`): The real-time distance between the two turtles.

- **Key Features**:
+ **Collision Avoidance**: If the turtles come too close (less than 1.5 meters), they are stopped to prevent collisions.
+ **Boundary Monitoring**: Prevents turtles from exiting the simulation area by stopping their motion if they approach the edges.

---

## Running the Package

### Option 1: Using the Launch File
The recommended method to run the simulation is by using the launch file:
```bash
roslaunch assignment1_rt assignment1.launch
```

This will automatically:
1. Start the Turtlesim simulation.
2. Launch the **UI** and **Distance** nodes.

By default, the second turtle will be spawned at `(7,7)` with an orientation of `0.0`. This can changed by modifying the following lines in the launch file: 
```xml
<!-- UI params -->
<param name="x" value="7.0" />
<param name="y" value="7.0" />
<param name="theta" value="0.0" />
```

### Option 2: Running Nodes Manually
If you prefer to launch the nodes individually:

1. Start the Turtlesim simulation:
```bash
rosrun turtlesim turtlesim_node
```

2. Run the **UI** Node : 

```bash
rosrun assignment1_rt UI.py
```

3. Run the **Distance** Node : 

```bash
rosrun assignment1_rt Distance.py
```

In this case also, the turtle is spawned by default at `(7,7)` with an orientation of `0.0`. To modify this, the **UI** node should be run with the following command: 

```bash
rosrun assignment1_rt UI.py _x:=3.0 _y:=4.0 _theta:=1.57
```

---

## Dependencies

Ensure the following dependencies are installed before running the package:
- **ROS**: The Robot Operating System (tested with ROS Noetic).
- **Turtlesim**: ROS package for the simulation environment.

---

## Files in the Package

- **`UI.py`**: Node to control turtles and spawn a second turtle.
- **`Distance.py`**: Node to calculate and monitor the distance between turtles.
- **`launch/assignment1.launch`**: Launch file to run the complete simulation.

---

## Additional Notes

- Use the **Turtlesim GUI** to visualize the turtles while sending commands.
- The `/relative_distance` topic can be echoed to observe the computed distances:

```bash
rostopic echo /relative_distance
```
  
- Ensure all files are marked as executables : 

```bash
chmod +x UI.py Distance.py
```

