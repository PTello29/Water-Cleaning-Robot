# Water-Cleaning Robot – Turtlesim Simulation

## Introduction

Safeguarding aquatic ecosystems has become a critical focus area in modern robotics and environmental engineering. As pollution levels rise in lakes, rivers, and coastal zones, autonomous solutions are increasingly sought after to mitigate human impact. This project introduces a simulated autonomous robot designed to identify and remove floating waste in a controlled virtual environment using the turtlesim package and ROS (Robot Operating System).

The robot operates in a 2D space, detecting a colored object that symbolizes surface-level debris. Upon detection, it navigates toward the target and simulates a cleaning action. Though simple in implementation, the behavior mirrors the fundamental logic behind real-world robots that use computer vision (e.g., OpenCV) and microcontroller integration to execute similar tasks.

This simulation is not only a technical experiment but also a conceptual contribution to sustainable development. By replicating environmental cleanup operations through automation, it aligns with key objectives in the United Nations 2030 Agenda, particularly the goal 6 of Clean Water and Sanitation that is supporting initiatives that reduce contamination in water bodies and the goal 14 ofLife Below Water  that is enhancing the conservation of aquatic biodiversity through pollution control.

Projects like this highlight the potential of robotics to assist in real-world ecological challenges while fostering awareness of technology’s role in environmental stewardship.

## System Overview

- The robot is implemented using the `turtlesim` package in ROS and programmed in Python.  
- A single virtual object (waste) is generated at a random position and assigned a fixed color.  
- The robot scans the field of view to detect the object.  
- Upon detection, the robot moves toward the waste and simulates a cleaning operation.  
- After completion, the robot is reset at the home position, and the process is repeated two more times.

## Theoretical Framework

### 1. ROS Architecture

ROS provides a distributed computing framework that allows various components to communicate efficiently. In this simulation, service calls handle turtle management, while topics manage movement and feedback.

### 2. Turtlesim

Turtlesim is a 2D simulator designed to teach ROS fundamentals. Though minimal, it is ideal for prototyping robot logic such as navigation, control loops, and perception simulation.

### 3. Proportional Control

The robot uses a proportional control loop to correct its heading based on the angular difference to the target. This method optimizes energy use by avoiding unnecessary motion until the robot is properly aligned.

### 4. Energy Consumption Metric

Rather than speed or timing, this simulation uses **energy consumption** as its primary performance metric. This is computed using the formula:

Energy $= \int (F \cdot v)dt$

Assuming constant force and approximating based on duration and velocity, energy consumption can be expressed as proportional to:

$E = k \cdot (v^2 + \omega^2) \cdot t$

Where:

- \( $v$ \) = linear velocity  
- \( $\omega$ \) = angular velocity  
- \( $t$ \) = total movement time  
- \( $k$ \) = arbitrary proportional constant  

This allows us to assess how efficiently the robot completes its task.

## Code Implementation and Explanation

```python
#!/usr/bin/env python3
import rospy, time, random
from math import atan2, sqrt, pow, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class EnergyRobot:
    def __init__(self):
        rospy.init_node("energy_cleaner")
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.energy_used = []

    def update_pose(self, data):
        self.pose = data

    def reset_robot(self):
        self.set_pen(0, 0, 0, 0, 1)
        self.teleport(1.0, 1.0, 0.0)
        self.set_pen(255, 255, 255, 3, 0)
        rospy.sleep(1)

    def spawn_trash(self):
        x = random.uniform(2.0, 10.0)
        y = random.uniform(2.0, 10.0)
        self.spawn(x, y, 0, 'trash')
        print(f"Trash spawned at position: x={x:.2f}, y={y:.2f}")  # Nuevo mensaje
        return (x, y)

    def move_to(self, x_goal, y_goal):
        vel = Twist()
        rate = rospy.Rate(10)
        k_linear = 0.5
        k_angular = 3.0
        start_time = time.perf_counter()
        energy = 0.0

        while not rospy.is_shutdown():
            dx = x_goal - self.pose.x
            dy = y_goal - self.pose.y
            dist = sqrt(dx**2 + dy**2)

            angle_to_goal = atan2(dy, dx)
            angle_error = angle_to_goal - self.pose.theta
            angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalize

            if dist < 0.1:
                break

            vel.linear.x = k_linear if abs(angle_error) < 0.1 else 0.0
            vel.angular.z = k_angular * angle_error
            self.vel_pub.publish(vel)

            # Approximate energy = v² + w²
            energy += pow(vel.linear.x, 2) + pow(vel.angular.z, 2)
            rate.sleep()

        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        end_time = time.perf_counter()

        total_energy = energy * (end_time - start_time) / 10
        self.energy_used.append(total_energy)
        self.kill("trash")

    def run(self, trials=3):
        for _ in range(trials):
            self.reset_robot()
            x, y = self.spawn_trash()
            rospy.sleep(1)
            self.move_to(x, y)

        print("Energy used per attempt:", [f"{e:.2f}" for e in self.energy_used])
        avg = sum(self.energy_used) / len(self.energy_used)
        print(f"Average energy: {avg:.2f}")

if __name__ == "__main__":
    try:
        robot = EnergyRobot()
        rospy.sleep(2)
        robot.run()
    except rospy.ROSInterruptException:
        pass
```
This Python script is designed to work with the **Turtlesim** simulator in ROS. It controls a virtual turtle that moves toward randomly spawned "trash" in the environment and estimates the energy used during each cleaning attempt.

## General Structure

The script includes:

- Importing necessary libraries
- Defining the `EnergyRobot` class
- Methods for control, motion, and logic
- A `main` block to execute the robot's behavior

### Initialization
```python
#!/usr/bin/env python3
import rospy, time, random
from math import atan2, sqrt, pow, pi
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class EnergyRobot:
    def __init__(self):
        rospy.init_node("energy_cleaner")
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.spawn = rospy.ServiceProxy('/spawn', Spawn)
        self.kill = rospy.ServiceProxy('/kill', Kill)
        self.teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
        self.set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
        self.energy_used = []
```
Sets up the ROS node and interfaces for controlling the turtle and interacting with services like spawn, kill, teleport, and pen color.

### update_pose
```python
def update_pose(self, data):
        self.pose = data
```
Callback function to update the turtle's current position.

### reset_robot
```python
def reset_robot(self):
        self.set_pen(0, 0, 0, 0, 1)
        self.teleport(1.0, 1.0, 0.0)
        self.set_pen(255, 255, 255, 3, 0)
        rospy.sleep(1)
```
Moves the turtle to a starting point and disables the pen trail.

### spawn_trash
```python
def spawn_trash(self):
        x = random.uniform(2.0, 10.0)
        y = random.uniform(2.0, 10.0)
        self.spawn(x, y, 0, 'trash')
        print(f"Trash spawned at position: x={x:.2f}, y={y:.2f}")  # Nuevo mensaje
        return (x, y)
```
Spawns a "trash" turtle at a random location within the simulation window.

### move_to
```python
def move_to(self, x_goal, y_goal):
        vel = Twist()
        rate = rospy.Rate(10)
        k_linear = 0.5
        k_angular = 3.0
        start_time = time.perf_counter()
        energy = 0.0

        while not rospy.is_shutdown():
            dx = x_goal - self.pose.x
            dy = y_goal - self.pose.y
            dist = sqrt(dx**2 + dy**2)

            angle_to_goal = atan2(dy, dx)
            angle_error = angle_to_goal - self.pose.theta
            angle_error = (angle_error + pi) % (2 * pi) - pi  # Normalize

            if dist < 0.1:
                break

            vel.linear.x = k_linear if abs(angle_error) < 0.1 else 0.0
            vel.angular.z = k_angular * angle_error
            self.vel_pub.publish(vel)

            # Approximate energy = v² + w²
            energy += pow(vel.linear.x, 2) + pow(vel.angular.z, 2)
            rate.sleep()

        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        end_time = time.perf_counter()

        total_energy = energy * (end_time - start_time) / 10
        self.energy_used.append(total_energy)
        self.kill("trash")
```
Navigates the turtle to the trash using proportional control for linear and angular velocity. Energy is estimated with `v² + w²`.

### run
```python
def run(self, trials=3):
        for _ in range(trials):
            self.reset_robot()
            x, y = self.spawn_trash()
            rospy.sleep(1)
            self.move_to(x, y)

        print("Energy used per attempt:", [f"{e:.2f}" for e in self.energy_used])
        avg = sum(self.energy_used) / len(self.energy_used)
        print(f"Average energy: {avg:.2f}")
```
Runs the cleaning process for a specified number of trials and prints the energy used.

## Energy Estimation

Energy is approximated by:

```
energy ≈ v² + w²
```

Where:
- `v`: linear velocity
- `w`: angular velocity

This provides a **relative measure** of energy consumed during motion.

## Performance Metric Evaluation

The simulation evaluates the robot's performance by estimating its energy usage during motion. Instead of prioritizing time-to-completion, this approach emphasizes efficient pathing and movement strategy.

### *1st Attempt*

![Energía gastada en la primera basura](/images/1.jpg)

- Energy consumed: **77.09 units**

### *2nd Attempt*

![Energía gastada en la primera basura](/images/2.jpg)

- Energy consumed: **123.17 units**

### *3rd Attempt*

![Energía gastada en la primera basura](/images/3.jpg)

- Energy consumed: **77.78 units**

**Average energy consumption:** **92.68 units**

![Energía gastada en la primera basura](/images/4.jpg)

## Justification

Variation in energy use is primarily influenced by two factors:

- **Initial orientation** relative to the waste location
- **Distance** from the robot's starting point

Shorter distances and near-perfect initial alignment result in lower energy usage. Higher angular corrections and longer paths significantly increase energy consumption.

This demonstrates the importance of efficient orientation and accurate path planning in autonomous systems, especially when energy optimization is a priority.

## Conclusions

This simulation project demonstrated how a virtual robot can be programmed to perform waste removal tasks efficiently in a controlled environment. By using energy usage as the main evaluation criterion, we introduced a performance perspective focused on sustainability and real-world constraints.

Key lessons include:

- Proportional control significantly reduces unnecessary movements.  
- Angular misalignment plays a larger role in energy inefficiency than distance.  

Future work may include introducing obstacles, dynamic targets, or even reinforcement learning techniques to further improve performance.

## References

1. "Pérez, L., & Rodriguez, J. (2020). Autonomous Robots for Environmental Monitoring: A Review. Robotics and Autonomous Systems, 127, 103472.
2. Wang, Y., & Lin, D. (2021). Simulation Environments for Robotics Education Using ROS and Turtlesim. International Journal of Engineering Education, 37(1), 55–62.
3. United Nations. (2015). Transforming our World: The 2030 Agenda for Sustainable Development. Retrieved from https://sdgs.un.org/goals
