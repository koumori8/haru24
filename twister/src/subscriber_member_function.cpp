#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Joy & joy) const
  {
    /*
    logicool f310
    axes
    0:leftaxe_left
    1:leftaxes_up
    2:LT
    3:rightaxes_left
    4:rightaxe_up
    5:RT
    6:dpad_left
    7:dpad_up
    buttons
    0:A
    1:B
    2:X
    3:Y
    4:LB
    5:RB
    6:back
    7:start
    8:logi(cannot use)
    9:leftaxe_push
    10:rightaxe_push
    */
    RCLCPP_INFO(this->get_logger(), "\nleftaxe_left:%lf \nleftaxe_up:%lf \nA:%d\nB:%d\nX:%d\nY:%d\nLB:%d\nRB:%d\nleftaxe_push:%d\nrightaxe_push:%d", joy.axes.at(0), joy.axes.at(1),joy.buttons.at(0), joy.buttons.at(1), joy.buttons.at(2), joy.buttons.at(3), joy.buttons.at(4), joy.buttons.at(5),  joy.buttons.at(9), joy.buttons.at(10));
  }
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
