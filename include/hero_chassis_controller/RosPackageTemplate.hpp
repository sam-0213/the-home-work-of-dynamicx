#pragma once

#include "hero_chassis_controller/Algorithm.hpp"

// ROS
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>

#include "Algorithm.hpp"
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <control_toolbox/ParametersConfig.h>
namespace hero_chassis_controller
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class RosPackageTemplate
{
 public:
  void update(const ros::Time& time, const ros::Duration& period);

  /*!
   * Constructor.
   *
   * @param nodeHandle the ROS node handle.
   */
  RosPackageTemplate(ros::NodeHandle &nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RosPackageTemplate();

 private:
  control_toolbox::Pid wheel_pids_[4];  // 4个轮子对应4个PID控制器
  double target_vel_[4];                // 4个轮子的目标速度
  double current_vel_[4];               // 4个轮子的当前速度（实际速度）

 // --- 用于 rqt_plot 画图的发布者 ---
  ros::Publisher target_pub_[4];
  ros::Publisher current_pub_[4];

 // --- 用于 rqt_reconfigure 动态调参的服务器 ---
  std::unique_ptr<dynamic_reconfigure::Server<control_toolbox::ParametersConfig>> dyn_server_;

 // 动态调参的回调函数声明
  void dynamicReconfigCallback(control_toolbox::ParametersConfig& config, uint32_t level);
  /*!
   * Reads and verifies the ROS parameters.
   *
   * @return true if successful.
   */
  
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void topicCallback(const sensor_msgs::Temperature &message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request &request,
                       std_srvs::Trigger::Response &response);

  //! ROS node handle.
  ros::NodeHandle &nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;

  //! ROS service server.
  ros::ServiceServer serviceServer_;

  //! Algorithm computation object.
  Algorithm algorithm_;
};

} /* namespace */
