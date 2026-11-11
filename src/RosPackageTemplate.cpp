#include "hero_chassis_controller/RosPackageTemplate.hpp"
#include <std_msgs/Float64.h>
// STD
#include <string>

namespace hero_chassis_controller
{

RosPackageTemplate::RosPackageTemplate(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle)
{
  for (int i = 0; i < 4; ++i) {
    target_vel_[i] = 0.0;
    current_vel_[i] = 0.0;
    target_pub_[i] = nodeHandle_.advertise<std_msgs::Float64>("wheel_" + std::to_string(i) + "/target_vel", 10);
    current_pub_[i] = nodeHandle_.advertise<std_msgs::Float64>("wheel_" + std::to_string(i) + "/current_vel", 10);
  }
  readParameters();
  dyn_server_ = std::make_unique<dynamic_reconfigure::Server<control_toolbox::ParametersConfig>>(nodeHandle_);
  dynamic_reconfigure::Server<control_toolbox::ParametersConfig>::CallbackType f;
  f = boost::bind(&RosPackageTemplate::dynamicReconfigCallback, this, _1, _2);
  dyn_server_->setCallback(f);

  ROS_INFO("Hero Chassis Controller initialized and Publishers ready.");

  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  subscriber_ = nodeHandle_.subscribe(subscriberTopic_, 1,
                                      &RosPackageTemplate::topicCallback, this);
  serviceServer_ = nodeHandle_.advertiseService("get_average",
                                                &RosPackageTemplate::serviceCallback, this);
  ROS_INFO("Successfully launched node.");
}

RosPackageTemplate::~RosPackageTemplate()
{
}

bool RosPackageTemplate::readParameters()
{

  for (int i = 0; i < 4; ++i) {
    double p, i_val, d;
    std::string prefix = "wheel_" + std::to_string(i);

    nodeHandle_.param(prefix + "/p", p, 1.0);
    nodeHandle_.param(prefix + "/i", i_val, 0.1);
    nodeHandle_.param(prefix + "/d", d, 0.01);

    wheel_pids_[i].initPid(p, i_val, d, 10.0, -10.0);
    ROS_INFO("Wheel %d PID: p=%.2f, i=%.2f, d=%.2f", i, p, i_val, d);
  }

  return true;
}




void RosPackageTemplate::dynamicReconfigCallback(control_toolbox::ParametersConfig& config, uint32_t level) {
  wheel_pids_[0].setGains(config.p, config.i, config.d, config.i_clamp_max, config.i_clamp_min);
  ROS_INFO("Reconfigure Request: p=%.2f, i=%.2f, d=%.2f", config.p, config.i, config.d);
}


void RosPackageTemplate::topicCallback(const sensor_msgs::Temperature &message)
{
  algorithm_.addData(message.temperature);
}

bool RosPackageTemplate::serviceCallback(std_srvs::Trigger::Request &request,
                                         std_srvs::Trigger::Response &response)
{
  response.success = true;
  response.message = "The average is " + std::to_string(algorithm_.getAverage());
  return true;
}
void RosPackageTemplate::update(const ros::Time& time, const ros::Duration& period) {
  double dt = period.toSec();
  if (dt <= 0.0) return;

  // 假设目标速度是阶跃信号：前 2 秒为 0，第 2 秒后突变为 2.0 rad/s (方便看超调)
  double sim_time = time.toSec();
  for (int i = 0; i < 4; ++i) {
    target_vel_[i] = (sim_time > 2.0) ? 2.0 : 0.0;
  }

  for (int i = 0; i < 4; ++i) {
    double error = target_vel_[i] - current_vel_[i];
    double cmd_effort = wheel_pids_[i].computeCommand(error, ros::Duration(dt));

    // 模拟电机响应过程
    current_vel_[i] += cmd_effort * dt;

    // 【新增】发布数据给 rqt_plot 画图
    std_msgs::Float64 target_msg, current_msg;
    target_msg.data = target_vel_[i];
    current_msg.data = current_vel_[i];
    target_pub_[i].publish(target_msg);
    current_pub_[i].publish(current_msg);

    // 终端日志保留，方便观察
    ROS_INFO("Wheel %d | Target: %.2f | Current: %.2f | Effort: %.2f",
             i, target_vel_[i], current_vel_[i], cmd_effort);
  }
}
} /* namespace */
