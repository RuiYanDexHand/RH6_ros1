#pragma once

// ROS1 headers
#include <ros/ros.h>

#include <string>

// Custom message (ROS1)
#include "rh6_cmd/Rh6Cmd.h"

namespace ruiyan {
namespace rh6 {

class rh6_test
{
public:
  // 构造函数：传入已有的 NodeHandle 以及节点名称（用于日志/命名等）
  explicit rh6_test(ros::NodeHandle& nh, const std::string& name);

  // 主动发布一次命令（也可由定时器触发）
  void PubCmd();

  // 角度/弧度转换工具
  static float rad_to_deg(float rad);
  static float deg_to_rad(float deg);

private:
  // 定时器回调：周期性发布命令
  void onTimer(const ros::TimerEvent&);

private:
  // 计数与周期
  int tick{0};
  int tspan_ms{10};

  // 要发布的命令消息缓存
  rh6_cmd::Rh6Cmd rh6cmd;

  // ROS1 通信对象
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher ryhand_cmd_publisher_;
};

} // namespace rh6
} // namespace ruiyan