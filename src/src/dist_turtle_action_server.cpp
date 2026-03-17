#include <iostream>

#include "rclcpp.h"
#include "turtlesim.h"
#include "geometry_msgs.h"
#include "my_second_package_msgs.h"
#include "my_second_package.h"
#include "rcl_interfaces.h"

using rclcpp::action;
using rclcpp::executors;
using rclcpp::node;

using turtlesim::msg;
using geometry_msgs::msg;
using my_second_package_msgs::action;
using my_second_package::my_subscriber;

using rcl_interfaces::msg;

#include <math.h>
#include <time.h>

class TurtleSub_Action: public TurtlesimSubscriber 
{
  public:
    TurtleSub_Action(ac_server)
    {
      this->ac_server = ac_server;
    }

  void callback(msg)
    {
      this->ac_server.current_pose = msg;
    }
};

class DistTurtleServer: public Node
{
  
}

