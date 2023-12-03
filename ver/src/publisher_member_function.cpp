#include <chrono>
#include <functional>
#include <memory>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "math.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

float theta = -M_PI/4; //robot orientation in global coordinates
const float R = 0.65 / M_SQRT2; //distance between the robot's center and a wheel
rclcpp::Time past = rclcpp::Clock().now();


class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "topic_twist", 10, std::bind(&MinimalPublisher::calc, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::String>("odrive_cmd", 10);
    }

  private:
    void calc(geometry_msgs::msg::Twist twist)
    {
      /*
      v.at(0):FR
      v.at(1):FL  
      v.at(2):BL
      v.at(3):BR
      */
      std_msgs::msg::String message; // velocity of wheels
      vector<float> v(4);
      float vx = twist.linear.x;
      float vy = twist.linear.y;
      rclcpp::Time now = rclcpp::Clock().now();
      theta += twist.angular.z * (now.nanoseconds() - past.nanoseconds())*1E-9;
      float vtheta = twist.angular.z;
      for (int i = 0; i < 4; i++) {
        v.at(i) = -sin(theta + M_PI/2 * i) * vx + cos(theta + M_PI/2 * i) * vy + R * vtheta;
      }
      RCLCPP_INFO(this->get_logger(), "FR%f  FL%f BL%f  BR%f", v.at(1), v.at(0), v.at(2), v.at(3), (now.nanoseconds() - past.nanoseconds())*1E-9);
      message.data = "FR" + to_string(v.at(0)) + "FL" + to_string(v.at(1)) + "BL" + to_string(v.at(2)) + "BR" + to_string(v.at(3));
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      past = now;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
