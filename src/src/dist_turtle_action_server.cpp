#include <memory>
#include <thread>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pse.hpp"
#include "my_second_package_msgs/action/dist_turtle.hpp"

using namespace std::chrono_literals;

class DistTurtleServer : public rclcpp::Node 
{
public:
  using DistTurtle = my_second_package_msgs::action::DistTurtle;
  using GoalHandleDistTurtle = rclcpp_action::ServerGoalHandle<DistTurtle>;
  
}