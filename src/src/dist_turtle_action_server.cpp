#include <memory>
#include <cmath>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_first_package_msgs/action/dist_turtle.hpp"

using namespace std::chrono_literals;

class DistTurtleServer : public rclcpp::Node
{
public:
  using DistTurtle = my_first_package_msgs::action::DistTurtle;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DistTurtle>;

  DistTurtleServer()
  : Node("dist_turtle_action_server"),
    total_dist_(0.0),
    is_first_time_(true)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10);

    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&DistTurtleServer::pose_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<DistTurtle>(
      this,
      "dist_turtle",
      std::bind(&DistTurtleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DistTurtleServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&DistTurtleServer::handle_accepted, this, std::placeholders::_1));

    this->declare_parameter("quatile_time", 0.75);
    this->declare_parameter("almost_goal_time", 0.95);

    quantile_time_ = this->get_parameter("quatile_time").as_double();
    almost_goal_time_ = this->get_parameter("almost_goal_time").as_double();

    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DistTurtleServer::param_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp_action::Server<DistTurtle>::SharedPtr action_server_;

  turtlesim::msg::Pose current_pose_;
  turtlesim::msg::Pose previous_pose_;

  double total_dist_;
  bool is_first_time_;

  double quantile_time_;
  double almost_goal_time_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    current_pose_ = *msg;
  }

  double calc_diff_pose()
  {
    if (is_first_time_) {
      previous_pose_ = current_pose_;
      is_first_time_ = false;
    }

    double dx = current_pose_.x - previous_pose_.x;
    double dy = current_pose_.y - previous_pose_.y;

    double dist = std::sqrt(dx * dx + dy * dy);

    previous_pose_ = current_pose_;
    return dist;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const DistTurtle::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Goal received");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{std::bind(&DistTurtleServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<DistTurtle::Feedback>();
    auto result = std::make_shared<DistTurtle::Result>();

    geometry_msgs::msg::Twist msg;
    msg.linear.x = goal_handle->get_goal()->linear_x;
    msg.angular.z = goal_handle->get_goal()->angular_z;

    rclcpp::Rate rate(100);

    while (rclcpp::ok()) {
      total_dist_ += calc_diff_pose();

      feedback->remained_dist =
        goal_handle->get_goal()->dist - total_dist_;

      goal_handle->publish_feedback(feedback);
      publisher_->publish(msg);

      if (feedback->remained_dist < 0.2) {
        break;
      }

      rate.sleep();
    }

    goal_handle->succeed(result);

    result->pos_x = current_pose_.x;
    result->pos_y = current_pose_.y;
    result->pos_theta = current_pose_.theta;
    result->result_dist = total_dist_;

    total_dist_ = 0.0;
    is_first_time_ = true;
  }

  rcl_interfaces::msg::SetParametersResult param_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & param : params) {
      RCLCPP_INFO(this->get_logger(),
        "%s changed to %f", param.get_name().c_str(), param.as_double());

      if (param.get_name() == "quatile_time") {
        quantile_time_ = param.as_double();
      }
      if (param.get_name() == "almost_goal_time") {
        almost_goal_time_ = param.as_double();
      }
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DistTurtleServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}