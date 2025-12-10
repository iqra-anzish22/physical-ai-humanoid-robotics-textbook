---
title: ROS 2 کی مثالیں اور مشقیں
sidebar_position: 3
description: ROS 2 ترقی کے لیے عملی مثالیں اور مشقیں
keywords: [ros2, examples, exercises, robotics, practice, tutorials]
---

import Exercise from '@site/src/components/Exercise/Exercise';

# ROS 2 کی مثالیں اور مشقیں

## مثال 1: سادہ پبلشر/سبسکرائبر

یہ مثال موضوعات کا استعمال کرتے ہوئے ROS 2 میں بنیادی رابطہ کے نمونہ کو ظاہر کرتی ہے۔

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

### مشق 1: پبلشر/سبسکرائبر توسیع

<Exercise
  title="اعلی درجے کا پبلشر/سبسکرائبر"
  difficulty="beginner"
  type="practical"
  instructions="بنیادی پبلشر/سبسکرائبر مثال کو پیغام کی فلٹرنگ کو شامل کرنے کے لیے توسیع دیں۔ ایک سبسکرائبر تخلیق کریں جو صرف مخصوص کی ورڈز پر مشتمل پیغامات کو ہی عمل کرتا ہے۔ خرابی کے ہینڈلنگ اور لاگنگ کی صلاحیتیں شامل کریں۔"
  expectedOutcome="طلباء ROS 2 رابطہ کے بنیادی نمونے کو سمجھیں گے اور خرابی کے ہینڈلنگ کے ساتھ مضبوط پبلشر/سبسکرائبر نوڈس نافذ کر سکیں گے۔"
/>

## مثال 2: سروس سرور/کلائنٹ

یہ مثال درخواست/جواب رابطہ کے لیے خدمات کو نافذ کرنا دکھاتی ہے۔

```python
# service_member_function.py
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService:

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

### مشق 2: کسٹم سروس کا نفاذ

<Exercise
  title="روبوٹ کنٹرول کے لیے کسٹم سروس"
  difficulty="intermediate"
  type="practical"
  instructions="ایک کسٹم سروس تخلیق کریں جو روبوٹ کی حرکت کے حکم (لکیری اور زاویہ ویلوسٹی) کو قبول کرتی ہے اور ہدف تک پہنچنے کا متوقع وقت لوٹاتی ہے۔ سروس سرور اور کلائنٹ نوڈس دونوں نافذ کریں۔"
  expectedOutcome="طلباء کسٹم خدمات کو تخلیق کرنے کو سمجھیں گے اور ROS 2 میں درخواست/جواب رابطہ کے نمونے نافذ کریں گے۔"
/>

## مثال 3: ایکشن سرور/کلائنٹ

ایکشنز طویل کاموں کے لیے فیڈ بیک اور ہدف کا انتظام فراہم کرتے ہیں۔

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

### مشق 3: نیوی گیشن ایکشن سرور

<Exercise
  title="روبوٹ نیوی گیشن ایکشن سرور"
  difficulty="advanced"
  type="practical"
  instructions="روبوٹ نیوی گیشن کے لیے ایک ایکشن سرور نافذ کریں جو ہدف کا پوز قبول کرتا ہے اور پیشرفت پر فیڈ بیک فراہم کرتا ہے۔ رکاوٹ کا پتہ لگانا، راستہ دوبارہ حساب، اور ہدف کی برداشت چیکنگ شامل کریں۔"
  expectedOutcome="طلباء پیچیدہ ایکشن سرورز کو نافذ کرنے کو سمجھیں گے جن میں فیڈ بیک اور بحالی کے برتاؤ شامل ہیں روبوٹکس ایپلیکیشنز کے لیے۔"
/>

## مثال 4: پیرامیٹر مینجمنٹ

پیرامیٹر مینجمنٹ ROS 2 نوڈس کی رن ٹائم کنفیگریشن کی اجازت دیتا ہے۔

```python
# parameter_node.py
import rclpy
from rclpy.node import Node

class ParameterNode:

    def __init__(self):
        super().__init__('parameter_node')

        # ڈیفالٹ ویلیوز کے ساتھ پیرامیٹر کا اعلان کریں
        self.declare_parameter('robot_name', 'turtlebot')
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('safety_distance', 0.3)

        # پیرامیٹر ویلیوز حاصل کریں
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value

        # پیرامیٹر کال بیک سیٹ کریں
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.get_logger().info(f'Initialized {self.robot_name} with max velocity {self.max_velocity}')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'max_velocity' and param.value > 2.0:
                return SetParametersResult(successful=False, reason='Max velocity too high')
        return SetParametersResult(successful=True)
```

### مشق 4: متحرک پیرامیٹر کنفیگریشن

<Exercise
  title="متحرک تبدیلی کا نظام"
  difficulty="intermediate"
  type="practical"
  instructions="ایک پیرامیٹر سرور تخلیق کریں جو روبوٹ کے برتاؤ کی متحرک تبدیلی کی اجازت دیتا ہے۔ پیرامیٹر کی توثیق اور متعدد نوڈس کے درمیان ا coordination جو پیرامیٹر شیئر کرتے ہیں نافذ کریں۔"
  expectedOutcome="طلباء ROS 2 میں پیرامیٹر مینجمنٹ کو سمجھیں گے اور متحرک کنفیگریشن سسٹم نافذ کر سکیں گے۔"
/>

## مثال 5: TF2 ٹرانسفورمیشنز

TF2 (ٹرانسفارم لائبریری) کوآرڈینیٹ فریم ٹرانسفورمیشنز کا نظم کرتی ہے۔

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

### مشق 5: ملٹی-روبوٹ TF سسٹم

<Exercise
  title="ملٹی-روبوٹ کوآرڈینیٹ سسٹم"
  difficulty="advanced"
  type="practical"
  instructions="ایک TF سسٹم ڈیزائن کریں جو ایک ہی ماحول میں کام کرنے والے متعدد روبوٹس کے لیے ہو۔ روبوٹ-مقامی فریم اور ایک مشترکہ دنیا کے فریم کے درمیان کوآرڈینیٹ ٹرانسفورمیشنز نافذ کریں۔"
  expectedOutcome="طلباء کوآرڈینیٹ ٹرانسفورمیشنز کے لیے TF2 کو سمجھیں گے اور ملٹی-روبوٹ کوآرڈینیٹ سسٹم ڈیزائن کر سکیں گے۔"
/>

## مشق 6: ROS 2 لانچ سسٹم

<Exercise
  title="پیچیدہ لانچ سسٹم"
  difficulty="intermediate"
  type="practical"
  instructions="ایک لانچ فائل تخلیق کریں جو مختلف کنفیگریشنز کے ساتھ متعدد نوڈس شروع کرتی ہے۔ پیرامیٹر لوڈنگ، نوڈ ری میپنگ، اور ماحولیاتی متغیرات کی بنیاد پر مشروط لانچ شامل کریں۔"
  expectedOutcome="طلباء ROS 2 لانچ سسٹم کو سمجھیں گے اور پیچیدہ لانچ کنفیگریشنز تخلیق کر سکیں گے۔"
/>

## مشق 7: ROS 2 ٹیسٹنگ فریم ورک

<Exercise
  title="یونٹ اور انٹیگریشن ٹیسٹنگ"
  difficulty="advanced"
  type="practical"
  instructions="gtest اور rclcpp کا استعمال کرتے ہوئے ROS 2 نوڈس کے لیے یونٹ ٹیسٹ نافذ کریں۔ نوڈس کے درمیان رابطہ کی تصدیق کرنے والے انٹیگریشن ٹیسٹ تخلیق کریں اور سسٹم کے برتاؤ کو ٹیسٹ کریں۔"
  expectedOutcome="طلباء ROS 2 ٹیسٹنگ میتھوڈولو جیز کو سمجھیں گے اور جامع ٹیسٹ سوٹس نافذ کر سکیں گے۔"
/>

## مشق 8: کارکردگی کی اصلاح

<Exercise
  title="ریل ٹائم کارکردگی ٹیوننگ"
  difficulty="advanced"
  type="practical"
  instructions="ROS 2 سسٹم کی کارکردگی کا تجزیہ اور اصلاح کریں۔ کوالٹی آف سروس (QoS) ترتیبات، پیغام تھروٹلنگ، اور وسائل کا نظم نافذ کریں۔"
  expectedOutcome="طلباء ROS 2 میں کارکردگی کے خیالات کو سمجھیں گے اور ریل ٹائم سسٹم کو بہتر بنانے کے قابل ہوں گے۔"
/>

## مشق 9: ROS 2 سیکورٹی نفاذ

<Exercise
  title="محفوظ رابطہ سیٹ اپ"
  difficulty="advanced"
  type="theoretical"
  instructions="تصدیق، اجازت، اور خفیہ کاری سمیت ROS 2 سیکورٹی کی خصوصیات کنفیگر کریں۔ ایک ملٹی-روبوٹ سسٹم کے لیے ایک سیکورٹی پالیسی ڈیزائن کریں۔"
  expectedOutcome="طلباء ROS 2 سیکورٹی کی خصوصیات کو سمجھیں گے اور روبوٹکس سسٹم میں محفوظ رابطہ نافذ کر سکیں گے۔"
/>

## مشق 10: ROS 1/ROS 2 برج

<Exercise
  title="ROS برج نفاذ"
  difficulty="advanced"
  type="practical"
  instructions="ROS 1 اور ROS 2 سسٹم کے درمیان ایک برج سیٹ کریں۔ دونوں سسٹم کے درمیان پیغامات منتقل کریں اور ڈیٹا ٹائپ تبادلوں کو سنبھالیں۔"
  expectedOutcome="طلباء ROS 1 اور ROS 2 سسٹم کو جوڑنے کو سمجھیں گے اور مختلف ROS ورژن کے درمیان مابینہ کاری کا نظم کر سکیں گے۔"
/>

## خلاصہ

یہ مثالیں اور مشقیں بنیادی ROS 2 تصورات کے ساتھ ہاتھوں سے تجربہ فراہم کرتی ہیں۔ ان نمونوں کو نافذ کرنا اور انہیں زیادہ پیچیدہ روبوٹکس ایپلیکیشنز تخلیق کرنے کے لیے توسیع دینا مشق کریں۔