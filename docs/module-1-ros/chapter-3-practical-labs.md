---
sidebar_position: 3
title: "Practical Labs: ROS 2 with rclpy"
---

# Practical Labs: ROS 2 with rclpy

## Concept Overview

This chapter provides hands-on practical exercises using rclpy, the Python client library for ROS 2. Through these labs, you'll gain experience implementing real-world robotic communication patterns and handling common robotics scenarios.

## Architecture Diagram: Lab Structure

```
┌─────────────────────────────────────────────────────────────────┐
│                ROS 2 Practical Lab Structure                    │
├─────────────────────────────────────────────────────────────────┤
│  Lab Components                                                 │
│  ┌─────────────────┬─────────────────┬─────────────────┐       │
│  │   Publisher     │   Subscriber    │   Service       │       │
│  │   Node          │   Node          │   Node          │       │
│  └─────────────────┴─────────────────┴─────────────────┘       │
│              │              │              │                   │
│              ▼              ▼              ▼                   │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │              Message Processing Logic                   │   │
│  │  ┌─────────────┬─────────────┬─────────────┬─────────┐ │   │
│  │  │  Validation │   Logging   │  Filtering  │Timing   │ │   │
│  │  │             │             │             │Control  │ │   │
│  │  └─────────────┴─────────────┴─────────────┴─────────┘ │   │
│  └─────────────────────────────────────────────────────────┘   │
│              │              │              │                   │
│              ▼              ▼              ▼                   │
│  ┌─────────────────┬─────────────────┬─────────────────┐       │
│  │   Sensor Data   │   Control       │   Feedback      │       │
│  │   Processing    │   Commands      │   Loop          │       │
│  └─────────────────┴─────────────────┴─────────────────┘       │
└─────────────────────────────────────────────────────────────────┘
```

## Lab 1: Basic Publisher/Subscriber with Custom Messages

### Creating a Custom Message

First, let's create a custom message for robot sensor data:

1. Create the message definition file `msg/RobotSensorData.msg` in your package:
   ```
   float64 linear_velocity
   float64 angular_velocity
   float64[] sensor_readings
   string sensor_type
   builtin_interfaces/Time timestamp
   ```

2. Update your package's `CMakeLists.txt` to include the message:
   ```cmake
   find_package(rosidl_default_generators REQUIRED)

   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/RobotSensorData.msg"
   )
   ```

### Publisher with Custom Message

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from my_robot_package.msg import RobotSensorData  # Custom message
import random
from builtin_interfaces.msg import Time

class RobotSensorPublisher(Node):
    def __init__(self):
        super().__init__('robot_sensor_publisher')
        self.publisher = self.create_publisher(RobotSensorData, 'robot_sensor_data', 10)

        # Timer to publish data at 1 Hz
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def timer_callback(self):
        msg = RobotSensorData()
        msg.linear_velocity = random.uniform(-1.0, 1.0)
        msg.angular_velocity = random.uniform(-0.5, 0.5)
        msg.sensor_readings = [random.uniform(0.0, 10.0) for _ in range(5)]
        msg.sensor_type = "LIDAR"

        # Set timestamp
        current_time = self.get_clock().now()
        msg.timestamp.sec = current_time.nanoseconds // 1000000000
        msg.timestamp.nanosec = current_time.nanoseconds % 1000000000

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: linear_vel={msg.linear_velocity:.2f}, '
                              f'angular_vel={msg.angular_velocity:.2f}, '
                              f'sensors={len(msg.sensor_readings)} readings')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    robot_sensor_publisher = RobotSensorPublisher()

    try:
        rclpy.spin(robot_sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        robot_sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber with Custom Message

```python
import rclpy
from rclpy.node import Node
from my_robot_package.msg import RobotSensorData  # Custom message

class RobotSensorSubscriber(Node):
    def __init__(self):
        super().__init__('robot_sensor_subscriber')
        self.subscription = self.create_subscription(
            RobotSensorData,
            'robot_sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: linear_vel={msg.linear_velocity:.2f}, '
                              f'angular_vel={msg.angular_velocity:.2f}, '
                              f'sensors={len(msg.sensor_readings)} readings')

def main(args=None):
    rclpy.init(args=args)
    robot_sensor_subscriber = RobotSensorSubscriber()

    try:
        rclpy.spin(robot_sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        robot_sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab 2: Parameter Server Usage

```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('sensor_enabled', True)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.sensor_enabled = self.get_parameter('sensor_enabled').value

        self.get_logger().info(f'Robot: {self.robot_name}, Max Vel: {self.max_velocity}, '
                              f'Safety Dist: {self.safety_distance}, Sensor: {self.sensor_enabled}')

        # Set up parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.type == ParameterType.PARAMETER_DOUBLE:
                self.get_logger().info(f'Updated max_velocity to: {param.value}')
                self.max_velocity = param.value
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    parameter_node = ParameterNode()

    try:
        rclpy.spin(parameter_node)
    except KeyboardInterrupt:
        pass
    finally:
        parameter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab 3: Lifecycle Nodes

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState
from std_msgs.msg import String

class LifecycleTalker(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_talker')
        self.pub = None

    def on_configure(self, state):
        self.get_logger().info(f'Configuring node: {state}')
        self.pub = self.create_publisher(String, 'lifecycle_chatter', 10)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info(f'Activating node: {state}')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info(f'Deactivating node: {state}')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info(f'Cleaning up node: {state}')
        self.destroy_publisher(self.pub)
        self.pub = None
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        if self.pub is not None and self.pub.get_subscription_count() > 0:
            msg = String()
            msg.data = f'Lifecycle talker: {self.get_clock().now().nanoseconds}'
            self.get_logger().info(f'Publishing: {msg.data}')
            self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    lifecycle_node = LifecycleTalker()

    # Create a timer to publish messages
    timer = lifecycle_node.create_timer(1.0, lifecycle_node.timer_callback)

    try:
        rclpy.spin(lifecycle_node)
    except KeyboardInterrupt:
        pass
    finally:
        lifecycle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lab Instructions

### Exercise 1: Implement a Robot Controller Node

Create a node that:
1. Subscribes to a "cmd_vel" topic to receive velocity commands
2. Publishes to a "robot_status" topic with current status
3. Uses parameters to configure maximum velocity limits
4. Implements proper error handling and logging

### Exercise 2: Create a Sensor Fusion Node

Create a node that:
1. Subscribes to multiple sensor topics (LIDAR, IMU, camera)
2. Fuses the sensor data into a single coherent representation
3. Publishes the fused data to a new topic
4. Handles different message types and timestamps

### Exercise 3: Parameter Configuration

1. Create a launch file that sets different parameter values
2. Run your parameter node with different configurations
3. Use `ros2 param` commands to change parameters at runtime
4. Observe how the node responds to parameter changes

## Humanoid URDF Example

For the humanoid robot mentioned in the tasks, here's a basic URDF structure:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.4"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.45"/>
  </joint>

  <!-- Arms -->
  <link name="left_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="torso_to_left_arm" type="revolute">
    <parent link="torso"/>
    <child link="left_arm"/>
    <origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Learning Objectives

After completing this chapter, you should be able to:
- Create and use custom message types in ROS 2
- Implement publisher and subscriber nodes with proper error handling
- Use ROS 2 parameters for configuration management
- Work with lifecycle nodes for more robust systems
- Create basic URDF models for humanoid robots
- Combine multiple ROS 2 concepts in practical applications