---
sidebar_position: 3
title: "Implementing Intelligent Behaviors"
---

# Implementing Intelligent Behaviors

## Concept Overview

Intelligent behaviors in robotics refer to the ability of robots to perceive their environment, make decisions based on that perception, and execute actions that achieve desired goals. This involves integrating perception, planning, control, and learning systems to create autonomous agents capable of operating in complex, dynamic environments. In this chapter, we'll explore how to implement intelligent behaviors using the Isaac ecosystem, including behavior trees, finite state machines, and learning-based approaches.

## Architecture Diagram: Intelligent Behavior System

```
┌─────────────────────────────────────────────────────────────────┐
│                Intelligent Behavior System                      │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────┐ │
│  │   Perception    │    │   Decision      │    │   Action    │ │
│  │   System        │───▶│   Making        │───▶│   Execution │ │
│  │   (Vision,      │    │   Engine        │    │   Layer     │ │
│  │   Sensors)      │    │   (BT, FSM, ML) │    │   (Motion,  │ │
│  └─────────────────┘    └─────────────────┘    │   Control)  │ │
│         │                       │               └─────────────┘ │
│         ▼                       ▼                      │       │
│  ┌─────────────────┐    ┌─────────────────┐           │       │
│  │   Environment   │    │   Learning      │           │       │
│  │   Model         │    │   System        │           │       │
│  │   (Maps,        │    │   (RL, Imitation│           │       │
│  │   Objects)      │    │   Learning)     │           │       │
│  └─────────────────┘    └─────────────────┘           ▼       │
│         │                       │               ┌─────────────┐│
│         └───────────────────────┼──────────────▶│   Feedback  ││
│                                 │               │   Loop      ││
│                                 └──────────────▶│   (Sensors,││
│                                                 │   Rewards)  ││
│                                                 └─────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

## Behavior Trees for Robotics

Behavior trees provide a powerful framework for organizing complex robot behaviors. They offer modularity, reusability, and the ability to handle complex decision-making scenarios.

### Basic Behavior Tree Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import time
import random

class BehaviorTree(Node):
    def __init__(self):
        super().__init__('behavior_tree')

        # Publishers and subscribers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Robot state
        self.obstacle_detected = False
        self.battery_level = 100.0
        self.task_completed = False

        # Create timer for behavior tree execution
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info('Behavior Tree Node initialized')

    def scan_callback(self, msg):
        """Process laser scan to detect obstacles."""
        # Check for obstacles in front of the robot (within 1 meter)
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < 1.0

    def tick(self):
        """Execute one tick of the behavior tree."""
        # Execute the root sequence
        result = self.root_sequence()

        if result == 'SUCCESS':
            self.get_logger().info('Behavior tree execution completed successfully')
        elif result == 'RUNNING':
            pass  # Still executing
        elif result == 'FAILURE':
            self.get_logger().error('Behavior tree execution failed')
        else:
            self.get_logger().warn(f'Unexpected behavior tree result: {result}')

    def root_sequence(self):
        """Root sequence of the behavior tree."""
        # Check if we need to recharge
        if self.check_battery_low():
            return self.execute_recharge_behavior()

        # If no obstacles, continue with main task
        if not self.obstacle_detected:
            if not self.task_completed:
                return self.execute_main_task()
            else:
                return 'SUCCESS'
        else:
            # Handle obstacle
            return self.handle_obstacle()

    def check_battery_low(self):
        """Check if battery level is low."""
        self.battery_level -= 0.1  # Simulate battery drain
        return self.battery_level < 20.0

    def execute_recharge_behavior(self):
        """Navigate to charging station."""
        self.get_logger().info('Battery low, navigating to charging station')

        # Publish goal to charging station
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = -5.0  # Charging station location
        goal_msg.pose.position.y = -5.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg.pose)
        return 'RUNNING'

    def execute_main_task(self):
        """Execute the main robot task."""
        self.get_logger().info('Executing main task')

        # Simulate task execution
        # In a real implementation, this would involve complex behaviors
        # like object manipulation, navigation to specific locations, etc.

        # For simulation, we'll complete the task after some time
        if random.random() > 0.99:  # Simulate task completion
            self.task_completed = True
            self.get_logger().info('Main task completed')
            return 'SUCCESS'

        return 'RUNNING'

    def handle_obstacle(self):
        """Handle obstacle avoidance."""
        self.get_logger().info('Obstacle detected, executing avoidance behavior')

        # Emergency stop if very close to obstacle
        if min(self.laser_data.ranges if hasattr(self, 'laser_data') else [2.0]) < 0.5:
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)
            return 'RUNNING'

        # Otherwise, navigate around obstacle
        # This would typically involve path planning algorithms
        return 'RUNNING'

    def reset_task(self):
        """Reset task completion for demonstration."""
        self.task_completed = False
        self.battery_level = 100.0
        self.get_logger().info('Task reset for demonstration')

def main(args=None):
    rclpy.init(args=args)
    bt_node = BehaviorTree()

    try:
        rclpy.spin(bt_node)
    except KeyboardInterrupt:
        pass
    finally:
        bt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Finite State Machine Implementation

Finite State Machines (FSMs) provide an alternative approach to organizing robot behaviors, particularly useful for simpler scenarios or when clear state transitions are needed.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class RobotFSM(Node):
    def __init__(self):
        super().__init__('robot_fsm')

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # FSM states
        self.STATES = {
            'IDLE': 0,
            'FORWARD': 1,
            'TURNING': 2,
            'OBSTACLE_AVOIDANCE': 3,
            'CHARGING': 4
        }

        self.current_state = self.STATES['IDLE']
        self.obstacle_distance = float('inf')
        self.state_start_time = time.time()

        # Create timer for FSM execution
        self.timer = self.create_timer(0.1, self.execute_fsm)

        self.get_logger().info('Robot FSM initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        if len(msg.ranges) > 0:
            self.obstacle_distance = min([r for r in msg.ranges if r > 0 and r < float('inf')], default=float('inf'))

    def execute_fsm(self):
        """Execute the finite state machine."""
        # Determine next state based on current state and sensor data
        next_state = self.determine_next_state()

        # Execute state transition if needed
        if next_state != self.current_state:
            self.execute_state_transition(next_state)

        # Execute current state behavior
        self.execute_current_state()

        # Publish current state
        state_msg = String()
        state_name = [name for name, value in self.STATES.items() if value == self.current_state][0]
        state_msg.data = state_name
        self.state_pub.publish(state_msg)

    def determine_next_state(self):
        """Determine the next state based on current state and sensor data."""
        current_state_name = [name for name, value in self.STATES.items() if value == self.current_state][0]

        if current_state_name == 'IDLE':
            if self.obstacle_distance > 2.0:  # No obstacles nearby
                return self.STATES['FORWARD']
            else:
                return self.STATES['OBSTACLE_AVOIDANCE']

        elif current_state_name == 'FORWARD':
            if self.obstacle_distance < 1.0:
                return self.STATES['OBSTACLE_AVOIDANCE']
            else:
                return self.STATES['FORWARD']

        elif current_state_name == 'OBSTACLE_AVOIDANCE':
            if self.obstacle_distance > 1.5:
                return self.STATES['FORWARD']
            else:
                return self.STATES['OBSTACLE_AVOIDANCE']

        elif current_state_name == 'TURNING':
            # Stay in turning state for a short duration
            if time.time() - self.state_start_time > 2.0:
                if self.obstacle_distance > 1.0:
                    return self.STATES['FORWARD']
                else:
                    return self.STATES['OBSTACLE_AVOIDANCE']
            else:
                return self.STATES['TURNING']

        else:  # Default to current state
            return self.current_state

    def execute_state_transition(self, next_state):
        """Execute actions required for state transition."""
        old_state = [name for name, value in self.STATES.items() if value == self.current_state][0]
        new_state = [name for name, value in self.STATES.items() if value == next_state][0]

        self.get_logger().info(f'Transitioning from {old_state} to {new_state}')

        # Stop robot when transitioning
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

        self.current_state = next_state
        self.state_start_time = time.time()

    def execute_current_state(self):
        """Execute behavior for the current state."""
        state_name = [name for name, value in self.STATES.items() if value == self.current_state][0]

        cmd_vel_msg = Twist()

        if state_name == 'IDLE':
            # Robot is idle, no movement
            pass

        elif state_name == 'FORWARD':
            # Move forward
            cmd_vel_msg.linear.x = 0.5
            cmd_vel_msg.angular.z = 0.0

        elif state_name == 'OBSTACLE_AVOIDANCE':
            # Simple obstacle avoidance - turn away from obstacle
            if self.obstacle_distance < 0.5:
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 1.0  # Turn left
            else:
                cmd_vel_msg.linear.x = 0.2
                cmd_vel_msg.angular.z = 0.5  # Gentle turn while moving

        elif state_name == 'TURNING':
            # Turn in place
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.8

        # Publish velocity command
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    fsm_node = RobotFSM()

    try:
        rclpy.spin(fsm_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop robot on shutdown
        stop_msg = Twist()
        fsm_node.cmd_vel_pub.publish(stop_msg)
        fsm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Learning-Based Behaviors with Reinforcement Learning

Reinforcement learning enables robots to learn complex behaviors through interaction with their environment. Isaac Lab provides tools for training RL agents.

### PPO-based Navigation Agent

```python
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import random

class PPOAgent(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(PPOAgent, self).__init__()

        # Actor network (policy)
        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()
        )

        # Critic network (value function)
        self.critic = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

        self.log_std = nn.Parameter(torch.zeros(action_dim))

    def forward(self, state):
        mu = self.actor(state)
        std = torch.exp(self.log_std)
        return mu, std

    def get_value(self, state):
        return self.critic(state)

    def get_action(self, state):
        mu, std = self.forward(state)
        dist = torch.distributions.Normal(mu, std)
        action = dist.sample()
        log_prob = dist.log_prob(action).sum(dim=-1)
        return action, log_prob

class RobotEnvironment:
    def __init__(self):
        # Initialize robot state
        self.robot_pos = np.array([0.0, 0.0])
        self.goal_pos = np.array([5.0, 5.0])
        self.obstacles = [
            np.array([2.0, 2.0]),
            np.array([3.0, 4.0])
        ]
        self.max_steps = 1000
        self.current_step = 0

    def reset(self):
        """Reset environment to initial state."""
        self.robot_pos = np.array([0.0, 0.0])
        self.current_step = 0
        return self.get_state()

    def get_state(self):
        """Get current state representation."""
        # State includes: robot position, goal direction, obstacle distances
        goal_direction = self.goal_pos - self.robot_pos
        goal_distance = np.linalg.norm(goal_direction)

        # Get distances to closest obstacles
        obstacle_distances = []
        for obs in self.obstacles:
            dist = np.linalg.norm(obs - self.robot_pos)
            obstacle_distances.append(dist)

        # Normalize and return state vector
        state = np.concatenate([
            self.robot_pos / 10.0,  # Normalized position
            goal_direction / 10.0,  # Normalized goal direction
            np.array(obstacle_distances) / 10.0,  # Normalized obstacle distances
            [goal_distance / 10.0]  # Normalized goal distance
        ])

        return state.astype(np.float32)

    def step(self, action):
        """Execute action and return (next_state, reward, done, info)."""
        # Apply action (velocity command)
        velocity = action * 0.5  # Scale action
        self.robot_pos += velocity

        # Calculate reward
        reward = self.calculate_reward()

        # Check termination conditions
        done = self.is_done()
        info = {}

        self.current_step += 1

        return self.get_state(), reward, done, info

    def calculate_reward(self):
        """Calculate reward based on current state."""
        # Distance to goal (negative reward for being far)
        goal_distance = np.linalg.norm(self.goal_pos - self.robot_pos)
        reward = -goal_distance * 0.1

        # Bonus for getting closer to goal
        if hasattr(self, '_prev_distance'):
            if goal_distance < self._prev_distance:
                reward += 0.1  # Small bonus for progress

        # Penalty for being near obstacles
        for obs in self.obstacles:
            obs_distance = np.linalg.norm(obs - self.robot_pos)
            if obs_distance < 0.5:
                reward -= 5.0  # Large penalty for being too close to obstacles

        # Bonus for reaching goal
        if goal_distance < 0.5:
            reward += 100.0  # Large bonus for reaching goal

        # Store previous distance for progress reward
        self._prev_distance = goal_distance

        return reward

    def is_done(self):
        """Check if episode is done."""
        goal_distance = np.linalg.norm(self.goal_pos - self.robot_pos)

        # Done if reached goal or exceeded max steps
        return (goal_distance < 0.5) or (self.current_step >= self.max_steps)

class PPOTrainer:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, eps_clip=0.2):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.agent = PPOAgent(state_dim, action_dim).to(self.device)
        self.optimizer = optim.Adam(self.agent.parameters(), lr=lr)

        self.gamma = gamma
        self.eps_clip = eps_clip
        self.buffer = []

    def select_action(self, state):
        """Select action using current policy."""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        action, log_prob = self.agent.get_action(state_tensor)
        value = self.agent.get_value(state_tensor)

        return action.cpu().numpy()[0], log_prob.cpu().item(), value.cpu().item()

    def update(self, states, actions, log_probs, rewards, values, dones):
        """Update policy using PPO algorithm."""
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        old_log_probs = torch.FloatTensor(log_probs).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        values = torch.FloatTensor(values).to(self.device)
        dones = torch.BoolTensor(dones).to(self.device)

        # Calculate advantages
        advantages = self.compute_advantages(rewards, values, dones)
        returns = advantages + values

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # Optimize policy
        for _ in range(4):  # PPO uses multiple epochs per update
            mu, std = self.agent.forward(states)
            dist = torch.distributions.Normal(mu, std)
            new_log_probs = dist.log_prob(actions).sum(dim=-1)

            # Calculate ratio
            ratio = torch.exp(new_log_probs - old_log_probs)

            # Calculate surrogate objectives
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.eps_clip, 1 + self.eps_clip) * advantages
            actor_loss = -torch.min(surr1, surr2).mean()

            # Value loss
            current_values = self.agent.get_value(states).squeeze()
            critic_loss = nn.MSELoss()(current_values, returns)

            # Total loss
            total_loss = actor_loss + 0.5 * critic_loss

            # Update
            self.optimizer.zero_grad()
            total_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.agent.parameters(), 0.5)
            self.optimizer.step()

    def compute_advantages(self, rewards, values, dones):
        """Compute advantages using Generalized Advantage Estimation."""
        advantages = torch.zeros_like(rewards).to(self.device)
        gae = 0

        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0 if dones[t] else values[t]
            else:
                next_value = values[t + 1]

            delta = rewards[t] + self.gamma * next_value * (1 - dones[t]) - values[t]
            gae = delta + self.gamma * 0.95 * (1 - dones[t]) * gae
            advantages[t] = gae

        return advantages

def train_rl_agent():
    """Train the RL agent for navigation."""
    env = RobotEnvironment()
    state_dim = 7  # Position (2) + Goal direction (2) + Obstacle distances (2) + Goal distance (1)
    action_dim = 2  # x, y velocity

    trainer = PPOTrainer(state_dim, action_dim)

    # Training loop
    for episode in range(1000):
        state = env.reset()
        episode_reward = 0
        episode_steps = 0

        states, actions, log_probs, rewards, values, dones = [], [], [], [], [], []

        # Collect trajectory
        for step in range(200):  # Collect 200 steps per episode
            action, log_prob, value = trainer.select_action(state)
            next_state, reward, done, _ = env.step(action)

            states.append(state)
            actions.append(action)
            log_probs.append(log_prob)
            rewards.append(reward)
            values.append(value)
            dones.append(done)

            state = next_state
            episode_reward += reward
            episode_steps += 1

            if done:
                break

        # Update policy
        trainer.update(states, actions, log_probs, rewards, values, dones)

        # Print progress
        if episode % 50 == 0:
            print(f"Episode {episode}, Reward: {episode_reward:.2f}, Steps: {episode_steps}")

    return trainer.agent

# Example usage in ROS node
class RLNavigationNode(Node):
    def __init__(self):
        super().__init__('rl_navigation_node')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Initialize RL agent
        self.state_dim = 7
        self.action_dim = 2
        self.rl_agent = PPOAgent(self.state_dim, self.action_dim)

        # Robot state
        self.scan_data = None
        self.robot_pos = np.array([0.0, 0.0])
        self.goal_pos = np.array([5.0, 5.0])

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('RL Navigation Node initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        self.scan_data = msg

    def get_robot_state(self):
        """Get current robot state for RL agent."""
        if self.scan_data is None:
            return np.zeros(self.state_dim, dtype=np.float32)

        # Simplified state representation
        goal_direction = self.goal_pos - self.robot_pos
        goal_distance = np.linalg.norm(goal_direction)

        # Use a few key laser ranges as obstacle representation
        front_range = min(self.scan_data.ranges[300:420]) if len(self.scan_data.ranges) > 420 else 10.0
        left_range = min(self.scan_data.ranges[150:210]) if len(self.scan_data.ranges) > 210 else 10.0
        right_range = min(self.scan_data.ranges[510:570]) if len(self.scan_data.ranges) > 570 else 10.0

        state = np.array([
            self.robot_pos[0] / 10.0,  # Normalized x position
            self.robot_pos[1] / 10.0,  # Normalized y position
            goal_direction[0] / 10.0,  # Normalized goal x direction
            goal_direction[1] / 10.0,  # Normalized goal y direction
            front_range / 10.0,        # Normalized front obstacle distance
            left_range / 10.0,         # Normalized left obstacle distance
            right_range / 10.0         # Normalized right obstacle distance
        ], dtype=np.float32)

        return state

    def control_loop(self):
        """Main control loop using RL agent."""
        if self.scan_data is not None:
            # Get current state
            state = self.get_robot_state()
            state_tensor = torch.FloatTensor(state).unsqueeze(0)

            # Get action from RL agent
            with torch.no_grad():
                mu, std = self.rl_agent(state_tensor)
                action = mu.numpy()[0]  # Use mean action

            # Convert action to velocity command
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = float(action[0]) * 0.5  # Scale action
            cmd_vel_msg.angular.z = float(action[1]) * 0.5  # Scale action

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    rl_node = RLNavigationNode()

    try:
        rclpy.spin(rl_node)
    except KeyboardInterrupt:
        pass
    finally:
        rl_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Sensor Data Processing for Intelligent Behaviors

Effective intelligent behaviors require proper processing of sensor data:

```python
import numpy as np
from scipy.spatial import distance
from sklearn.cluster import DBSCAN
import cv2

class SensorProcessor:
    def __init__(self):
        self.laser_data = None
        self.camera_data = None
        self.imu_data = None

    def process_laser_scan(self, laser_msg):
        """Process laser scan for obstacle detection and mapping."""
        ranges = np.array(laser_msg.ranges)
        angles = np.linspace(laser_msg.angle_min, laser_msg.angle_max, len(ranges))

        # Filter out invalid ranges
        valid_indices = (ranges > laser_msg.range_min) & (ranges < laser_msg.range_max) & (ranges > 0)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]

        # Convert to Cartesian coordinates
        x_points = valid_ranges * np.cos(valid_angles)
        y_points = valid_ranges * np.sin(valid_angles)

        # Cluster points to identify obstacles
        points = np.column_stack((x_points, y_points))

        if len(points) > 0:
            clustering = DBSCAN(eps=0.3, min_samples=5).fit(points)
            labels = clustering.labels_

            obstacles = []
            for label in set(labels):
                if label != -1:  # -1 indicates noise
                    cluster_points = points[labels == label]
                    center = np.mean(cluster_points, axis=0)
                    size = np.std(cluster_points, axis=0)
                    obstacles.append({
                        'center': center,
                        'size': size,
                        'points': cluster_points
                    })

            return obstacles
        else:
            return []

    def process_camera_data(self, image_msg):
        """Process camera image for object detection."""
        # Convert ROS image to OpenCV
        np_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(
            image_msg.height, image_msg.width, -1
        )

        # Simple color-based object detection (example)
        hsv = cv2.cvtColor(np_image, cv2.COLOR_RGB2HSV)

        # Define color range for detection (e.g., red objects)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([170, 50, 50])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter small contours
                # Calculate bounding box
                x, y, w, h = cv2.boundingRect(contour)
                objects.append({
                    'bbox': (x, y, w, h),
                    'area': area,
                    'center': (x + w//2, y + h//2)
                })

        return objects

    def integrate_sensor_data(self, laser_obstacles, camera_objects):
        """Integrate data from multiple sensors."""
        # Create a unified representation of the environment
        environment_map = {
            'obstacles': laser_obstacles,
            'objects_of_interest': camera_objects,
            'navigation_goals': []
        }

        # Example: fuse laser and camera data
        # This would involve coordinate transformations and data association
        for obj in camera_objects:
            # Convert image coordinates to world coordinates
            # This requires camera calibration and robot pose
            world_coords = self.image_to_world(obj['center'])

            # Associate with laser obstacles if they correspond
            closest_obstacle = self.find_closest_obstacle(laser_obstacles, world_coords)

            if closest_obstacle and distance.euclidean(closest_obstacle['center'], world_coords) < 0.5:
                # This camera object corresponds to a laser-detected obstacle
                closest_obstacle['visual_info'] = obj

        return environment_map

    def find_closest_obstacle(self, obstacles, point):
        """Find the closest obstacle to a given point."""
        if not obstacles:
            return None

        closest = obstacles[0]
        min_dist = distance.euclidean(closest['center'], point)

        for obstacle in obstacles[1:]:
            dist = distance.euclidean(obstacle['center'], point)
            if dist < min_dist:
                min_dist = dist
                closest = obstacle

        return closest if min_dist < 2.0 else None  # Only return if within 2m

    def image_to_world(self, image_coords):
        """Convert image coordinates to world coordinates."""
        # This requires camera calibration parameters and robot pose
        # Simplified implementation - in practice, this involves:
        # 1. Camera intrinsic parameters
        # 2. Robot pose in world frame
        # 3. Transformation matrices

        # For now, return a placeholder
        return np.array([image_coords[0] * 0.01, image_coords[1] * 0.01, 0.0])  # Simplified conversion
```

## Lab Instructions

### Exercise 1: Behavior Tree Implementation

1. Implement a complex behavior tree that includes:
   - Navigation to multiple waypoints
   - Object detection and manipulation
   - Battery monitoring and recharging
   - Obstacle avoidance

2. Test the behavior tree in simulation with various scenarios
3. Evaluate the robustness of the behavior tree under different conditions

### Exercise 2: Reinforcement Learning Navigation

1. Train an RL agent for navigation in a simulated environment
2. Compare the performance with traditional navigation approaches
3. Evaluate the agent's ability to generalize to new environments

### Exercise 3: Multi-Sensor Fusion

1. Implement sensor fusion between camera and LIDAR data
2. Create a unified environment representation
3. Test the system's ability to detect and track objects

## Best Practices for Intelligent Behaviors

### Design Principles
- **Modularity**: Keep behaviors modular and reusable
- **Safety**: Always include safety checks and emergency stops
- **Robustness**: Handle sensor failures and unexpected situations
- **Scalability**: Design systems that can handle increasing complexity

### Performance Considerations
- **Real-time constraints**: Ensure behaviors execute within time limits
- **Computational efficiency**: Optimize algorithms for real-time execution
- **Memory management**: Be mindful of memory usage in embedded systems

## Learning Objectives

After completing this chapter, you should be able to:
- Design and implement behavior trees for complex robot behaviors
- Create finite state machines for reactive robot control
- Train reinforcement learning agents for robot navigation
- Integrate multiple sensor modalities for intelligent decision making
- Implement sensor processing pipelines for perception-based behaviors
- Apply best practices for developing robust intelligent robot behaviors