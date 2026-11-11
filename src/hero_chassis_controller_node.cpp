#include <ros/ros.h>
#include "hero_chassis_controller/RosPackageTemplate.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hero_chassis_controller");
  ros::NodeHandle nodeHandle("~");

  hero_chassis_controller::RosPackageTemplate rosPackageTemplate(nodeHandle);
  ros::Rate rate(50);
  ros::Time last_time = ros::Time::now();

  // 3. 核心：死循环，不断地跑 PID
  while (ros::ok())
  {
    ros::Time now = ros::Time::now();
    ros::Duration period = now - last_time;
    last_time = now;

    // 调用 update 函数计算 PID 并发布数据
    rosPackageTemplate.update(now, period);

    ros::spinOnce(); // 处理一下可能有的回调
    rate.sleep();    // 休眠，保证 50Hz
  }

  return 0;
}
