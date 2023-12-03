#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
const float SPEEDGAIN = 1.0;
const float ANGULARSPEEDGAIN = 1.0;
using namespace std;
using std::placeholders::_1;

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
    twist.linear.x = SPEEDGAIN * joy.axes.at(0);
    twist.linear.y = SPEEDGAIN * joy.axes.at(1);
    twist.angular.z = ANGULARSPEEDGAIN * (joy.buttons.at(4) - joy.buttons.at(5));
    RCLCPP_INFO(this->get_logger(), "\n twist.linear.x: %f\n twist.linear.y: %f\n twist.angular.z:%f", twist.linear.x, twist.linear.y, twist.angular.z); 
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
