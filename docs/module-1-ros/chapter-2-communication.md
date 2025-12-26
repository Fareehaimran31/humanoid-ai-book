---
sidebar_position: 2
title: "ROS 2 Communication Patterns"
---

# ROS 2 Communication Patterns

## Concept Overview

ROS 2 communication patterns define how nodes interact with each other. Understanding these patterns is crucial for designing effective robotic systems. The main communication patterns in ROS 2 are topics (publish/subscribe), services (request/reply), and actions (goal/feedback/result).

## Communication Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                 ROS 2 Communication Patterns                    │
├─────────────────────────────────────────────────────────────────┤
│  Topics (Pub/Sub)                                               │
│  ┌─────────┐     ┌─────────┐     ┌─────────┐                   │
│  │Publisher│────▶│ Message │────▶│Subscriber│                   │
│  │   A     │     │  Bus    │     │    B    │                   │
│  └─────────┘     └─────────┘     └─────────┘                   │
│                                                                 │
│  Services (Req/Rep)                                             │
│  ┌─────────┐     ┌─────────┐     ┌─────────┐                   │
│  │ Client  │────▶│ Request │────▶│ Server  │                   │
│  │    A    │     │         │     │    B    │                   │
│  └─────────┘     └─────────┘     └─────────┘                   │
│         ▲                            │                         │
│         └─────────── Result ─────────┘                         │
│                                                                 │
│  Actions (Goal/Feedback/Result)                                 │
│  ┌─────────┐     ┌─────────┐     ┌─────────┐                   │
│  │ Client  │────▶│  Goal   │────▶│ Server  │                   │
│  │    A    │     │         │     │    B    │                   │
│  └─────────┘     └─────────┘     └─────────┘                   │
│         ▲         ▲         ▲         │                         │
│         └─Result──┘ └Feedback┘ └─Cancel─┘                       │
└─────────────────────────────────────────────────────────────────┘
```

## Topics - Publish/Subscribe Pattern

Topics enable asynchronous communication between nodes. Publishers send messages to a topic, and subscribers receive messages from a topic. This pattern is ideal for continuous data streams like sensor data.

### Creating a Subscriber Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services - Request/Reply Pattern

Services provide synchronous communication for request-response interactions. A client sends a request to a service server and waits for a response. This pattern is suitable for tasks that have a clear beginning and end.

### Creating a Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions - Goal/Feedback/Result Pattern

Actions are designed for long-running tasks that may be canceled and provide feedback during execution. This pattern is ideal for navigation, manipulation, and other complex tasks.

### Creating an Action Server

```python
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)
    fibonacci_action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ROS 2 Workspace Setup Guide

### Creating a ROS 2 Package

1. Navigate to your workspace source directory:
   ```bash
   cd ~/robotics_ws/src
   ```

2. Create a new package with dependencies:
   ```bash
   ros2 pkg create --build-type ament_python my_robot_package --dependencies rclpy std_msgs example_interfaces
   ```

3. Navigate to the package directory and update the setup.py file to include entry points for your nodes:
   ```python
   entry_points={
       'console_scripts': [
           'publisher = my_robot_package.publisher:main',
           'subscriber = my_robot_package.subscriber:main',
           'service_server = my_robot_package.service_server:main',
           'service_client = my_robot_package.service_client:main',
       ],
   },
   ```

## Lab Instructions

### Exercise 1: Publisher/Subscriber Communication

1. Create both publisher and subscriber nodes in the same package
2. Run the publisher in one terminal:
   ```bash
   ros2 run my_robot_package publisher
   ```
3. Run the subscriber in another terminal:
   ```bash
   ros2 run my_robot_package subscriber
   ```
4. Observe the communication between nodes

### Exercise 2: Service Communication

1. Create a service server and client in the same package
2. Run the service server:
   ```bash
   ros2 run my_robot_package service_server
   ```
3. Call the service from another terminal:
   ```bash
   ros2 service call /add_two_ints example_interfaces/AddTwoInts "{a: 2, b: 3}"
   ```
4. Observe the request/response communication

## Learning Objectives

After completing this chapter, you should be able to:
- Implement publisher and subscriber nodes for asynchronous communication
- Create service servers and clients for synchronous communication
- Understand when to use each communication pattern
- Set up a ROS 2 package with proper dependencies
- Test communication between nodes