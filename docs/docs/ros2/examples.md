---
title: ROS 2 Examples and Exercises
sidebar_position: 3
description: Practical examples and exercises for ROS 2 development
keywords: [ros2, examples, exercises, robotics, practice, tutorials]
---

import Exercise from '@site/src/components/Exercise/Exercise';

# ROS 2 Examples and Exercises

## Example 1: Simple Publisher/Subscriber

This example demonstrates the basic communication pattern in ROS 2 using topics.

```cpp
// publisher_member_function.cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};
```

### Exercise 1: Publisher/Subscriber Extension

<Exercise
  title="Enhanced Publisher/Subscriber"
  difficulty="beginner"
  type="practical"
  instructions="Extend the basic publisher/subscriber example to include message filtering. Create a subscriber that only processes messages containing specific keywords. Add error handling and logging capabilities."
  expectedOutcome="Students will understand basic ROS 2 communication patterns and be able to implement robust publisher/subscriber nodes with error handling."
/>

## Example 2: Service Server/Client

This example shows how to implement services for request/reply communication.

```python
# service_member_function.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

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

### Exercise 2: Custom Service Implementation

<Exercise
  title="Custom Service for Robot Control"
  difficulty="intermediate"
  type="practical"
  instructions="Create a custom service that accepts robot movement commands (linear and angular velocity) and returns the expected time to reach a target. Implement both the service server and client nodes."
  expectedOutcome="Students will understand how to create custom services and implement request/reply communication patterns in ROS 2."
/>

## Example 3: Action Server/Client

Actions provide feedback and goal management for long-running tasks.

```cpp
// action_server.cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "example_interfaces/action/fibonacci.hpp"

class FibonacciActionServer : public rclcpp::Node
{
public:
    using Fibonacci = example_interfaces::action::Fibonacci;
    using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

    explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("fibonacci_action_server", options)
    {
        using namespace std::placeholders;

        action_server_ = rclcpp_action::create_server<Fibonacci>(
            this,
            "fibonacci",
            std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
            std::bind(&FibonacciActionServer::handle_cancel, this, _1),
            std::bind(&FibonacciActionServer::handle_accepted, this, _1));
    }

private:
    rclcpp_action::ActionServer<Fibonacci>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Fibonacci::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&FibonacciActionServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Fibonacci::Feedback>();
        auto & sequence = feedback->sequence;
        auto result = std::make_shared<Fibonacci::Result>();

        sequence.push_back(0);
        sequence.push_back(1);

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
            if (goal_handle->is_canceling()) {
                result->sequence = sequence;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                return;
            }
            sequence.push_back(sequence[i] + sequence[i - 1]);
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Publishing feedback: %" PRId64, sequence[i + 1]);
            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            result->sequence = sequence;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
};
```

### Exercise 3: Navigation Action Server

<Exercise
  title="Robot Navigation Action Server"
  difficulty="advanced"
  type="practical"
  instructions="Implement an action server for robot navigation that accepts a goal pose and provides feedback on progress. Include obstacle detection, path recalculation, and goal tolerance checking."
  expectedOutcome="Students will understand how to implement complex action servers with feedback and recovery behaviors for robotics applications."
/>

## Example 4: Parameter Management

Parameter management allows runtime configuration of ROS 2 nodes.

```python
# parameter_node.py
import rclpy
from rclpy.node import Node

class ParameterNode(Node):

    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('safety_distance', 0.3)

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Initialized {self.robot_name} with max velocity {self.max_velocity}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.value > 2.0:
                return SetParametersResult(successful=False, reason='Max velocity too high')
        return SetParametersResult(successful=True)
```

### Exercise 4: Dynamic Parameter Configuration

<Exercise
  title="Dynamic Reconfiguration System"
  difficulty="intermediate"
  type="practical"
  instructions="Create a parameter server that allows dynamic reconfiguration of robot behaviors. Implement parameter validation and coordination between multiple nodes that share parameters."
  expectedOutcome="Students will understand parameter management in ROS 2 and be able to implement dynamic configuration systems."
/>

## Example 5: TF2 Transformations

TF2 (Transform Library) manages coordinate frame transformations.

```cpp
// tf2_example.cpp
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TF2Example : public rclcpp::Node
{
public:
    TF2Example() : Node("tf2_example")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
```

### Exercise 5: Multi-Robot TF System

<Exercise
  title="Multi-Robot Coordinate System"
  difficulty="advanced"
  type="practical"
  instructions="Design a TF system for multiple robots operating in the same environment. Implement coordinate transformations between robot-local frames and a shared world frame."
  expectedOutcome="Students will understand TF2 for coordinate transformations and be able to design multi-robot coordinate systems."
/>

## Exercise 6: ROS 2 Launch System

<Exercise
  title="Complex Launch System"
  difficulty="intermediate"
  type="practical"
  instructions="Create a launch file that starts multiple nodes with different configurations. Include parameter loading, node remapping, and conditional launching based on environment variables."
  expectedOutcome="Students will understand the ROS 2 launch system and be able to create complex launch configurations."
/>

## Exercise 7: ROS 2 Testing Framework

<Exercise
  title="Unit and Integration Testing"
  difficulty="advanced"
  type="practical"
  instructions="Implement unit tests for ROS 2 nodes using gtest and rclcpp. Create integration tests that verify communication between nodes and test system behavior."
  expectedOutcome="Students will understand ROS 2 testing methodologies and be able to implement comprehensive test suites."
/>

## Exercise 8: Performance Optimization

<Exercise
  title="Real-Time Performance Tuning"
  difficulty="advanced"
  type="practical"
  instructions="Analyze and optimize the performance of a ROS 2 system. Implement Quality of Service (QoS) settings, message throttling, and resource management."
  expectedOutcome="Students will understand performance considerations in ROS 2 and be able to optimize real-time systems."
/>

## Exercise 9: ROS 2 Security Implementation

<Exercise
  title="Secure Communication Setup"
  difficulty="advanced"
  type="theoretical"
  instructions="Configure ROS 2 security features including authentication, authorization, and encryption. Design a security policy for a multi-robot system."
  expectedOutcome="Students will understand ROS 2 security features and be able to implement secure communication in robotics systems."
/>

## Exercise 10: ROS 1/ROS 2 Bridge

<Exercise
  title="ROS Bridge Implementation"
  difficulty="advanced"
  type="practical"
  instructions="Set up a bridge between ROS 1 and ROS 2 systems. Transfer messages between the two systems and handle data type conversions."
  expectedOutcome="Students will understand how to connect ROS 1 and ROS 2 systems and manage inter-operability between different ROS versions."
/>

## Summary

These examples and exercises provide hands-on experience with core ROS 2 concepts. Practice implementing these patterns and extending them to create more complex robotics applications.