#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std;
using std::placeholders::_1;

const float SPEEDGAIN = 0.05;
const float ANGULARSPEEDGAIN = 0.15;
const float RC = 1.0; //Response Curve of the controller (probably 1.0 ~ 1.2)
bool automatic = true;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalPublisher::calc, this, _1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("topic_twist", 10);
  }

private:
  void calc(const sensor_msgs::msg::Joy & joy)
  {
    auto twist = geometry_msgs::msg::Twist(); 
   
    if (joy.buttons.at(0) == 1) {
      automatic = true;
    }
    if (joy.buttons.at(1) == 1) {
      automatic = false;
    }

    if (automatic == false) { 
      if (joy.axes.at(0) == 0) twist.linear.x = 0;
      else twist.linear.x = -1 * SPEEDGAIN * pow(abs(joy.axes.at(0)), RC) * (joy.axes.at(0) / abs(joy.axes.at(0)));
      if (joy.axes.at(1) == 0) twist.linear.y = 0;
      else twist.linear.y = SPEEDGAIN * pow(abs(joy.axes.at(1)), RC) * (joy.axes.at(1) / abs(joy.axes.at(1)));
      twist.angular.z = ANGULARSPEEDGAIN * (joy.buttons.at(4) - joy.buttons.at(5));
    }

    if (automatic == true) {
      twist.linear.x = 0;
      twist.linear.y = 0;
      twist.angular.z = 0;
    }

    RCLCPP_INFO(this->get_logger(), "\n twist.linear.x: %f\n twist.linear.y: %f\n twist.angular.z:%f ", twist.linear.x, twist.linear.y, twist.angular.z); 
    publisher_->publish(twist); 
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

/*
axes
0 left stick(left)
1 left stick(up)

buttons
0 A
1 B
2 X
3 Y
4 LB
5 RB
*/
