#define BOOST_MPL_LIMIT_LIST_SIZE 50
#define BOOST_VARIANT_LIMIT_TYPES 50
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#include "rh6_test.hpp"
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <string>
#include <chrono>
#include <cmath>

#define MOTOR_NUM 6

namespace ruiyan {
namespace rh6 {

rh6_test::rh6_test(ros::NodeHandle& nh, const std::string& name)
: nh_(nh)
{
  ROS_INFO("hello %s (ROS1)", name.c_str());

  // 初始化命令
  rh6cmd = rh6_cmd::Rh6Cmd();
  rh6cmd.mode = 0;
  rh6cmd.lr = 0;
  tick = 0;
  tspan_ms = 5;

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    rh6cmd.m_pos[i] = 4095;
    rh6cmd.m_spd[i] = 1000;
    rh6cmd.m_curlimit[i] = 1000;
  }

  // 创建发布器 - 命令
  ryhand_cmd_publisher_ = nh_.advertise<rh6_cmd::Rh6Cmd>("ryhand6_cmd", 1);

  // 创建定时器 tspan_ms ms
  timer_ = nh_.createTimer(ros::Duration(tspan_ms * 0.001), &rh6_test::onTimer, this);
}

float rh6_test::rad_to_deg(float rad)
{
  return static_cast<float>(rad * 180.0 / M_PI);
}

float rh6_test::deg_to_rad(float deg)
{
  return static_cast<float>(deg * M_PI / 180.0);
}

void rh6_test::PubCmd()
{
  float p1, p2;       // p3,p4; // 保留
  float period = 10000;   // 周期 ms
  float amplitude = 1500; // 振幅为 1500
  float fs = std::sin(2 * M_PI * tick / period);
  float fc = std::cos(2 * M_PI * tick / period);

  // 主动序列
  std::vector<int> active_sequence = {0, 1, 3, 5, 7, 9};

  switch (rh6cmd.mode)
  {
    // 原始电机命令
    case 0:
      for (int i = 0; i < MOTOR_NUM; i++)
      {
        p1 = amplitude + amplitude * fs;
        p2 = 1000 * 1000 * (amplitude * 4) / 4095 * fc / period + 800;  // 600+

        if (i <= 1)
        {
          rh6cmd.m_pos[i] = 0;
          rh6cmd.m_spd[i] = 1000;
        }
        else
        {
          rh6cmd.m_pos[i] = p1;
          rh6cmd.m_spd[i] = p2;
        }
      }
      break;

    // 关节角（弧度）命令
    case 1:
      for (int i = 0; i < 11; i++)
      {
        if (std::find(active_sequence.begin(), active_sequence.end(), i) != active_sequence.end())
        {
          if (i == 0)      p1 = 40 + 40 * fs;
          else if (i == 1) p1 = 10 + 10 * fs;
          else              p1 = 30 + 30 * fs;
          rh6cmd.j_ang[i] = deg_to_rad(p1);
        }
        else
        {
          rh6cmd.j_ang[i] = rh6cmd.j_ang[i - 1];
        }
      }
      break;

    // 末端位姿命令（占位，保持接口对齐，不做修改）
    case 2:
      break;

    default:
      break;
  }

  tick = (tick + tspan_ms) % 100000;
  ryhand_cmd_publisher_.publish(rh6cmd);
}

void rh6_test::onTimer(const ros::TimerEvent&)
{
  PubCmd();
}

} // namespace rh6
} // namespace ruiyan

// ROS1 main，保持与现有结构对齐，不引入多余逻辑
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "rh6_test");
  ros::NodeHandle nh("~");

  try
  {
    ruiyan::rh6::rh6_test node(nh, "rh6_test");
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
  }

  return 0;
}