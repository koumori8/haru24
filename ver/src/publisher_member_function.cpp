#include <chrono>
#include <functional>
#include <memory>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "math.h"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

float theta = -M_PI/4; //robot orientation in global coordinates
const float R = 0.5; //distance between the robot's center and a wheel
rclcpp::Time past = rclcpp::Clock().now();


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "topic_twist", 10, std::bind(&MinimalPublisher::calc, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_velocity", 10);
    }

  private:
    void calc(geometry_msgs::msg::Twist twist)
    {
      /*
      v.at(0):RF
      v.at(1):LF  
      v.at(2):LB
      v.at(3):RB
      */
      std_msgs::msg::Float32MultiArray v; // velocity of wheels
      vector<float> alpha(4);
      float vx = twist.linear.x;
      float vy = twist.linear.y;
      rclcpp::Time now = rclcpp::Clock().now();
      theta += twist.angular.z * (now.nanoseconds() - past.nanoseconds())*1E-9;
      float vtheta = twist.angular.z;
      for (int i = 0; i < 4; i++) {
        float velocity = -sin(theta + M_PI/2 * i) * vx + cos(theta + M_PI/2 * i) * vy + R * vtheta;
        v.data.push_back(velocity);
      }
      RCLCPP_INFO(this->get_logger(), "\nLF:%f  RF:%f\nLB:%f  RB:%f %f", v.data.at(1), v.data.at(0), v.data.at(2), v.data.at(3), (now.nanoseconds() - past.nanoseconds())*1E-9);
      publisher_->publish(v);
      past = now;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
