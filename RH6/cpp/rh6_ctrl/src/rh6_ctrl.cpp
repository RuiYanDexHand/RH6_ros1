#define BOOST_MPL_LIMIT_LIST_SIZE 50
#define BOOST_VARIANT_LIMIT_TYPES 50
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
// e:\ros代码\RH6\cpp_ros1\rh6_ctrl\src\rh6_ctrl.cpp
#include "rh6_ctrl.hpp"

#include <algorithm>
#include <vector>
#include <cmath>
#include <cstring>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#ifdef __linux__
  #include <pthread.h>
  #include <sched.h>
  #include <unistd.h>
#endif

#define MOTOR_NUM 6

extern "C" {
  #include "ryhandlib_port.h"   // 若未提供实现，可临时不包含并使用本地工具函数
  #include "can_socket.h"
}

namespace {
// 本地工具函数（避免与 C 接口符号重名导致二义性）
inline float local_rad_to_deg(float rad) { return static_cast<float>(rad * 180.0f / static_cast<float>(M_PI)); }
inline float local_deg_to_rad(float deg) { return static_cast<float>(deg * static_cast<float>(M_PI) / 180.0f); }
inline double local_evaluatePolynomial(double coefficients[], int degree, double x) {
  double result = 0.0;
  for (int i = 0; i <= degree; ++i) {
    result = result * x + coefficients[i];
  }
  return result;
}
} // namespace

namespace ruiyan {
namespace rh6 {

rh6_ctrl::rh6_ctrl(ros::NodeHandle& nh, const std::string& name)
: nh_(nh)
{
    ROS_INFO("hello %s (ROS1 Noetic)", name.c_str());

    si = 0;
    rh6msg = rh6_msg::Rh6Msg();
    rh6cmd = rh6_cmd::Rh6Cmd();

    // 多项式系数（与现有 ROS2 版本一致）
    // 拇指
    poly_coeff[0][0] = 0.000329;
    poly_coeff[0][1] = -0.035054;
    poly_coeff[0][2] = 2.558963;
    poly_coeff[0][3] = 0.272863;

    // 食指
    poly_coeff[1][0] = 0.000010;
    poly_coeff[1][1] = -0.004996;
    poly_coeff[1][2] = 1.426094;
    poly_coeff[1][3] = -0.044273;

    // 中指
    poly_coeff[2][0] = 0.000002;
    poly_coeff[2][1] = -0.002910;
    poly_coeff[2][2] = 1.283182;
    poly_coeff[2][3] = -0.088568;

    // 无名指
    poly_coeff[3][0] = 0.000010;
    poly_coeff[3][1] = -0.004996;
    poly_coeff[3][2] = 1.426094;
    poly_coeff[3][3] = -0.044273;

    // 小指
    poly_coeff[4][0] = 0.000016;
    poly_coeff[4][1] = -0.006612;
    poly_coeff[4][2] = 1.529302;
    poly_coeff[4][3] = -0.011082;

    // URDF 路径和指尖命名
    std::string pkg_path = ros::package::getPath("rh6_ctrl");
    urdf_path = pkg_path + "/urdf";

    urdf_filename_l = urdf_path + "/ruihand6z.urdf";
    fingertip_l_[0] = "fz14";
    fingertip_l_[1] = "fz23";
    fingertip_l_[2] = "fz33";
    fingertip_l_[3] = "fz43";
    fingertip_l_[4] = "fz53";

    pinocchio::urdf::buildModel(urdf_filename_l, model_l_);
    data_l_ = pinocchio::Data(model_l_);

    urdf_filename_r = urdf_path + "/ruihand6y.urdf";
    fingertip_r_[0] = "fy14";
    fingertip_r_[1] = "fy23";
    fingertip_r_[2] = "fy33";
    fingertip_r_[3] = "fy43";
    fingertip_r_[4] = "fy53";

    pinocchio::urdf::buildModel(urdf_filename_r, model_r_);
    data_r_ = pinocchio::Data(model_r_);

    // 关节向量
    q_.resize(model_l_.nq);    q_.setZero();
    q_fk_.resize(model_l_.nq); q_fk_.setZero();
    q_ik_.resize(model_l_.nq); q_ik_.setZero();
    q_iik_.resize(model_l_.nq);q_iik_.setZero();

    ROS_INFO("joint_num: %d", (int)model_l_.nq);
    ROS_INFO("urdf_l_path: %s", urdf_filename_l.c_str());
    ROS_INFO("urdf_r_path: %s", urdf_filename_r.c_str());
    ROS_INFO("end: %s. id=%d", fingertip_r_[0].c_str(), (int)model_r_.getJointId(fingertip_r_[0]));

    // 参数
    std::string pub_name, sub_name, control_type;
    nh_.param<std::string>("ryhand_pub_topic_name", pub_name, std::string("ryhand6_status"));
    nh_.param<std::string>("ryhand_sub_topic_name", sub_name, std::string("ryhand6_cmd"));
    nh_.param<std::string>("control_type", control_type, std::string("normal"));
    ROS_INFO("control_type = %s", control_type.c_str());

    if (control_type == "normal") {
      ryhand_state_publisher_ = nh_.advertise<rh6_msg::Rh6Msg>(pub_name, 10);
      ryhand_cmd_subscriber_ = nh_.subscribe(sub_name, 10, &rh6_ctrl::CmdCallback, this);
      timer_ = nh_.createTimer(ros::Duration(0.01), &rh6_ctrl::onTimer, this); // 10ms
    }

    // ROS1 服务
    server_rh6fk_ = nh_.advertiseService("rh6_fk", &rh6_ctrl::rh6fk_callback, this);
    server_rh6ik_ = nh_.advertiseService("rh6_ik", &rh6_ctrl::rh6ik_callback, this);
}

void rh6_ctrl::CmdCallback(const rh6_cmd::Rh6Cmd::ConstPtr& msg)
{
  auto cmd = *msg;
  rh6msg.lr = cmd.lr;

  // 将电机命令写入待发缓冲
  for (int i = 0; i < MOTOR_NUM; ++i) {
    sutServoDataW[i].stuCmd.usTp = cmd.m_pos[i];
    sutServoDataW[i].stuCmd.usTv = cmd.m_spd[i];
    sutServoDataW[i].stuCmd.usTc = cmd.m_curlimit[i];
  }

  switch (cmd.mode) {
    case 2: { // 末端位姿到关节角（IK）
      // 选择左右手模型
      if (rh6msg.lr) {
        model_ = model_r_; data_ = data_r_;
        fingertip[0] = fingertip_r_[0]; fingertip[1] = fingertip_r_[1]; fingertip[2] = fingertip_r_[2];
        fingertip[3] = fingertip_r_[3]; fingertip[4] = fingertip_r_[4];
      } else {
        model_ = model_l_; data_ = data_l_;
        fingertip[0] = fingertip_l_[0]; fingertip[1] = fingertip_l_[1]; fingertip[2] = fingertip_l_[2];
        fingertip[3] = fingertip_l_[3]; fingertip[4] = fingertip_l_[4];
      }

      // 填充请求并调用本地 IK 函数
      rh6_cmd::Rh6ik::Request req; rh6_cmd::Rh6ik::Response res;
      req.lr = rh6msg.lr;
      req.x_base = cmd.x_base; req.y_base = cmd.y_base; req.z_base = cmd.z_base;
      req.roll_base = cmd.roll_base; req.pitch_base = cmd.pitch_base; req.yaw_base = cmd.yaw_base;
      for (int i = 0; i < 5; ++i) {
        req.x[i] = cmd.x[i]; req.y[i] = cmd.y[i]; req.z[i] = cmd.z[i];
        req.roll[i] = cmd.roll[i]; req.pitch[i] = cmd.pitch[i]; req.yaw[i] = cmd.yaw[i];
      }
      rh6ik(model_, data_, q_iik_, fingertip, req, res);

      // 将关节角映射为电机目标值
      for (int i = 0; i < 11; ++i) cmd.j_ang[i] = res.j_ang[i];
      for (int i = 0; i < 11; ++i) {
        switch (i) {
          case 0:  sutServoDataW[0].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(135.0f)); break;
          case 1:  sutServoDataW[1].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(40.0f));  break;
          case 3:  sutServoDataW[2].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(87.0f));  break;
          case 5:  sutServoDataW[3].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(90.0f));  break;
          case 7:  sutServoDataW[4].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(90.0f));  break;
          case 9:  sutServoDataW[5].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(88.5f));break;
          default: break;
        }
      }
      UpdataMotor();
      break;
    }

    case 1: { // 直接关节角到电机（弧度）
      for (int i = 0; i < 11; ++i) {
        switch (i) {
          case 0:  sutServoDataW[0].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(135.0f)); break;
          case 1:  sutServoDataW[1].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(40.0f));  break;
          case 3:  sutServoDataW[2].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(87.0f));  break;
          case 5:  sutServoDataW[3].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(90.0f));  break;
          case 7:  sutServoDataW[4].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(90.0f));  break;
          case 9:  sutServoDataW[5].stuCmd.usTp = radx_to_cmd(cmd.j_ang[i], local_deg_to_rad(88.5f));break;
          default: break;
        }
      }
      UpdataMotor();
      break;
    }

    default: { // 0 原始电机命令
      UpdataMotor();
      break;
    }
  }

  rh6cmd = cmd; // 缓存最后一次命令
}

void rh6_ctrl::onTimer(const ros::TimerEvent&)
{
  PubState();
}

void rh6_ctrl::PubState()
{
  int p, v, t;

  // 状态采集与关节角计算
  for (int i = 0; i < MOTOR_NUM; ++i) {
    rh6msg.status[i] = sutServoDataR[i].stuInfo.ucStatus;

    p = sutServoDataR[i].stuInfo.ub_P;
    v = sutServoDataR[i].stuInfo.ub_V;
    t = sutServoDataR[i].stuInfo.ub_I;
    if (v > 2047) v -= 4096;
    if (t > 2047) t -= 4096;

    rh6msg.m_pos[i] = static_cast<float>(p);
    rh6msg.m_spd[i] = static_cast<float>(v);
    rh6msg.m_cur[i] = static_cast<float>(t);
    rh6msg.m_force[i] = sutServoDataR[i].stuInfo.ub_F;

    // 角度映射（弧度）
    switch (i) {
      case 0:
        rh6msg.j_ang[0] = cmd_to_radx(p, local_deg_to_rad(135.0f));
        break;
      case 1:
        rh6msg.j_ang[1] = cmd_to_radx(p, local_deg_to_rad(40.0f));
        rh6msg.j_ang[2] = ::deg_to_rad(evaluatePolynomial(poly_coeff[0], 3, ::rad_to_deg(static_cast<float>(rh6msg.j_ang[1]))));
        break;
      case 2:
        rh6msg.j_ang[3] = cmd_to_radx(p, local_deg_to_rad(87.0f));
        rh6msg.j_ang[4] = ::deg_to_rad(evaluatePolynomial(poly_coeff[1], 3, ::rad_to_deg(static_cast<float>(rh6msg.j_ang[3]))));
        break;
      case 3:
        rh6msg.j_ang[5] = cmd_to_radx(p, local_deg_to_rad(90.0f));
        rh6msg.j_ang[6] = ::deg_to_rad(evaluatePolynomial(poly_coeff[2], 3, ::rad_to_deg(static_cast<float>(rh6msg.j_ang[5]))));
        break;
      case 4:
        rh6msg.j_ang[7] = cmd_to_radx(p, local_deg_to_rad(90.0f));
        rh6msg.j_ang[8] = ::deg_to_rad(evaluatePolynomial(poly_coeff[3], 3, ::rad_to_deg(static_cast<float>(rh6msg.j_ang[7]))));
        break;
      case 5:
        rh6msg.j_ang[9] = cmd_to_radx(p, local_deg_to_rad(88.5f));
        rh6msg.j_ang[10] = ::deg_to_rad(evaluatePolynomial(poly_coeff[4], 3, ::rad_to_deg(static_cast<float>(rh6msg.j_ang[9]))));
        break;
      default:
        break;
    }
  }

  // 选择左右手模型
  if (rh6msg.lr) {
    model_ = model_r_;
    data_  = data_r_;
    fingertip[0] = fingertip_r_[0];
    fingertip[1] = fingertip_r_[1];
    fingertip[2] = fingertip_r_[2];
    fingertip[3] = fingertip_r_[3];
    fingertip[4] = fingertip_r_[4];
  } else {
    model_ = model_l_;
    data_  = data_l_;
    fingertip[0] = fingertip_l_[0];
    fingertip[1] = fingertip_l_[1];
    fingertip[2] = fingertip_l_[2];
    fingertip[3] = fingertip_l_[3];
    fingertip[4] = fingertip_l_[4];
  }

  // 将当前关节角写入模型并做限幅
  std::array<float, 11> req_j{};
  for (int i = 0; i < 11; ++i) req_j[i] = rh6msg.j_ang[i];
  req_angle_to_model_q(req_j, q_, static_cast<int>(model_.nq));
  for (int i = 0; i < q_.size(); ++i) {
    q_[i] = std::clamp(q_[i], model_.lowerPositionLimit[i], model_.upperPositionLimit[i]);
  }

  // 前向运动学并变换至世界系
  pinocchio::forwardKinematics(model_, data_, q_);

  pinocchio::SE3 ee1 = data_.oMi[ model_.getJointId(fingertip[0]) ];
  pinocchio::SE3 ee2 = data_.oMi[ model_.getJointId(fingertip[1]) ];
  pinocchio::SE3 ee3 = data_.oMi[ model_.getJointId(fingertip[2]) ];
  pinocchio::SE3 ee4 = data_.oMi[ model_.getJointId(fingertip[3]) ];
  pinocchio::SE3 ee5 = data_.oMi[ model_.getJointId(fingertip[4]) ];

  rh6msg.x_base = rh6cmd.x_base;
  rh6msg.y_base = rh6cmd.y_base;
  rh6msg.z_base = rh6cmd.z_base;
  rh6msg.roll_base = rh6cmd.roll_base;
  rh6msg.pitch_base = rh6cmd.pitch_base;
  rh6msg.yaw_base = rh6cmd.yaw_base;

  Eigen::Vector3d tbase(rh6msg.x_base, rh6msg.y_base, rh6msg.z_base);
  Eigen::Quaterniond q_base =
      Eigen::AngleAxisd(rh6msg.roll_base,  Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(rh6msg.pitch_base, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rh6msg.yaw_base,   Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d Rbase = q_base.toRotationMatrix();
  pinocchio::SE3 Tbw(Rbase, tbase);

  std::vector<pinocchio::SE3> E = { Tbw*ee1, Tbw*ee2, Tbw*ee3, Tbw*ee4, Tbw*ee5 };
  for (int i = 0; i < 5; ++i) {
    rh6msg.x[i] = E[i].translation().x();
    rh6msg.y[i] = E[i].translation().y();
    rh6msg.z[i] = E[i].translation().z();

    Eigen::Quaterniond q(E[i].rotation());
    rh6msg.w[i] = q.w();
    rh6msg.i[i] = q.x();
    rh6msg.j[i] = q.y();
    rh6msg.k[i] = q.z();

    Eigen::Vector3d rpy = E[i].rotation().eulerAngles(0,1,2);
    rh6msg.roll[i]  = rpy[0];
    rh6msg.pitch[i] = rpy[1];
    rh6msg.yaw[i]   = rpy[2];
  }

  // 发布
  ryhand_state_publisher_.publish(rh6msg);
}

void rh6_ctrl::UpdataMotor()
{
  for (int i = 0; i < MOTOR_NUM; ++i) {
    RyMotion_ServoMove_Mix(&stuServoCan,
                           i + 1,
                           sutServoDataW[i].stuCmd.usTp,
                           sutServoDataW[i].stuCmd.usTv,
                           sutServoDataW[i].stuCmd.usTc,
                           &sutServoDataR[i],
                           1);
  }
}

void rh6_ctrl::req_angle_to_model_q(const std::array<float, 11>& req, Eigen::VectorXd& q, int size)
{
  for (int i = 0; i < size; i++)
  {
    switch (i)
    {
      // 拇指
      case 0: q[0] = req[0]; break;
      case 1: q[1] = req[1]; break;
      case 2: q[2] = deg_to_rad(evaluatePolynomial(poly_coeff[0], 3, rad_to_deg(q[i - 1]))); break;
      case 3: q[3] = 0; break;

      // 食指
      case 4: q[4] = req[3]; break;
      case 5: q[5] = deg_to_rad(evaluatePolynomial(poly_coeff[1], 3, rad_to_deg(q[i - 1]))); break;
      case 6: q[6] = 0; break;

      // 中指
      case 7: q[7] = req[5]; break;
      case 8: q[8] = deg_to_rad(evaluatePolynomial(poly_coeff[2], 3, rad_to_deg(q[i - 1]))); break;
      case 9: q[9] = 0; break;

      // 无名指
      case 10: q[10] = req[7]; break;
      case 11: q[11] = deg_to_rad(evaluatePolynomial(poly_coeff[3], 3, rad_to_deg(q[i - 1]))); break;
      case 12: q[12] = 0; break;

      // 小指
      case 13: q[13] = req[9]; break;
      case 14: q[14] = deg_to_rad(evaluatePolynomial(poly_coeff[4], 3, rad_to_deg(q[i - 1]))); break;
      case 15: q[15] = 0; break;

      default: break;
    }
  }
}

void rh6_ctrl::model_q_to_res_angle(const Eigen::VectorXd& q, std::array<float, 11>& res)
{
  const std::vector<std::pair<int, int>> coupleds = {
    {0,0},{1,1},{2,2},{3,4},{4,5},{5,7},{6,8},{7,10},{8,11},{9,13},{10,14}
  };
  for (size_t i = 0; i < coupleds.size(); i++) {
    auto [dest, src] = coupleds[i];
    res[dest] = static_cast<float>(q[src]);
  }
}

bool rh6_ctrl::rh6fk_callback(rh6_cmd::Rh6fk::Request& req, rh6_cmd::Rh6fk::Response& res)
{
  ROS_INFO("rh6fk");

  if (req.lr) {
    model_fk_ = model_r_;
    data_fk_ = data_r_;
    fingertip_fk[0] = fingertip_r_[0];
    fingertip_fk[1] = fingertip_r_[1];
    fingertip_fk[2] = fingertip_r_[2];
    fingertip_fk[3] = fingertip_r_[3];
    fingertip_fk[4] = fingertip_r_[4];
  } else {
    model_fk_ = model_l_;
    data_fk_ = data_l_;
    fingertip_fk[0] = fingertip_l_[0];
    fingertip_fk[1] = fingertip_l_[1];
    fingertip_fk[2] = fingertip_l_[2];
    fingertip_fk[3] = fingertip_l_[3];
    fingertip_fk[4] = fingertip_l_[4];
  }

  q_fk_.resize(model_fk_.nq);
  q_fk_.setZero();

  // 注意：这里假设 req.j_ang 为“弧度”，如为“度”请先转换为弧度
  std::array<float, 11> req_j{};
  for (int i = 0; i < 11; ++i) req_j[i] = req.j_ang[i];
  req_angle_to_model_q(req_j, q_fk_, static_cast<int>(model_fk_.nq));

  for (int i = 0; i < q_fk_.size(); i++) {
    q_fk_[i] = std::max(model_fk_.lowerPositionLimit[i], std::min(q_fk_[i], model_fk_.upperPositionLimit[i]));
  }

  pinocchio::forwardKinematics(model_fk_, data_fk_, q_fk_);

  pinocchio::SE3 ee1 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[0] ) ];
  pinocchio::SE3 ee2 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[1] ) ];
  pinocchio::SE3 ee3 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[2] ) ];
  pinocchio::SE3 ee4 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[3] ) ];
  pinocchio::SE3 ee5 = data_fk_.oMi[ model_fk_.getJointId( fingertip_fk[4] ) ];

  Eigen::Vector3d t(req.x_base, req.y_base, req.z_base);
  Eigen::Quaterniond q_base =
      Eigen::AngleAxisd(req.roll_base,  Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(req.pitch_base, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(req.yaw_base,   Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = q_base.toRotationMatrix();
  pinocchio::SE3 Tbw(R, t);

  std::vector<pinocchio::SE3> ees = {
    Tbw*ee1, Tbw*ee2, Tbw*ee3, Tbw*ee4, Tbw*ee5
  };

  for (int i = 0; i < 5; ++i) {
    res.x[i] = ees[i].translation().x();
    res.y[i] = ees[i].translation().y();
    res.z[i] = ees[i].translation().z();

    Eigen::Quaterniond q(ees[i].rotation());
    res.w[i] = q.w();
    res.i[i] = q.x();
    res.j[i] = q.y();
    res.k[i] = q.z();

    Eigen::Vector3d rpy = ees[i].rotation().eulerAngles(0,1,2);
    res.roll[i]  = rpy[0];
    res.pitch[i] = rpy[1];
    res.yaw[i]   = rpy[2];
  }

  return true;
}

bool rh6_ctrl::rh6ik_callback(rh6_cmd::Rh6ik::Request& req, rh6_cmd::Rh6ik::Response& res)
{
  ROS_INFO("rh6ik");

  if (req.lr) {
    model_ik_ = model_r_;
    data_ik_ = data_r_;
    fingertip_ik[0] = fingertip_r_[0];
    fingertip_ik[1] = fingertip_r_[1];
    fingertip_ik[2] = fingertip_r_[2];
    fingertip_ik[3] = fingertip_r_[3];
    fingertip_ik[4] = fingertip_r_[4];
  } else {
    model_ik_ = model_l_;
    data_ik_ = data_l_;
    fingertip_ik[0] = fingertip_l_[0];
    fingertip_ik[1] = fingertip_l_[1];
    fingertip_ik[2] = fingertip_l_[2];
    fingertip_ik[3] = fingertip_l_[3];
    fingertip_ik[4] = fingertip_l_[4];
  }

  // 使用与 ROS2 一致的 IK 求解逻辑
  rh6ik(model_ik_, data_ik_, q_ik_, fingertip_ik, req, res);
  return true;
}

void rh6_ctrl::rh6ik(pinocchio::Model& model,
                     pinocchio::Data& /*data*/,
                     Eigen::VectorXd& q_ik,
                     std::string ftip[5],
                     const rh6_cmd::Rh6ik::Request& request,
                     rh6_cmd::Rh6ik::Response& response)
{
  // 基座到世界的位姿
  Eigen::Vector3d t(request.x_base, request.y_base, request.z_base);
  Eigen::Quaterniond q_base =
      Eigen::AngleAxisd(request.roll_base,  Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(request.pitch_base, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(request.yaw_base,   Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R = q_base.toRotationMatrix();
  pinocchio::SE3 Tbw(R, t);

  // 目标末端位姿（世界系）并换算到基座系
  pinocchio::SE3 target_poses[5];
  for (int i = 0; i < 5; ++i) {
    Eigen::Quaterniond q_i =
        Eigen::AngleAxisd(request.roll[i],  Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(request.pitch[i], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(request.yaw[i],   Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d Ri = q_i.toRotationMatrix();
    Eigen::Vector3d pi(request.x[i], request.y[i], request.z[i]);
    pinocchio::SE3 oMi(Ri, pi);
    target_poses[i] = Tbw.actInv(oMi); // 变换到基座
  }

  // 关节耦合与固定
  const std::vector<std::pair<int,int>> coupled = {{1,2},{4,5},{7,8},{10,11},{13,14}};
  const std::vector<int> fixed = {3,6,9,12,15};
  const Eigen::VectorXd minq = model.lowerPositionLimit;
  const Eigen::VectorXd maxq = model.upperPositionLimit;

  // 初始位姿
  q_ik = pinocchio::neutral(model);

  const double eps = 1e-4;
  const int IT_MAX = 50;
  const double DT = 0.5;
  const double damp = 1e-6;

  // 单指 IK 求解（顺序求解，易读稳定）
  auto solve_finger_ik = [&](int finger_idx, const pinocchio::SE3& oMdes, int target_joint_id, Eigen::VectorXd& qvec)
  {
    pinocchio::Data local_data(model);
    pinocchio::Data::Matrix6x J(6, model.nv); J.setZero();
    Eigen::VectorXd v(model.nv); v.setZero();
    Eigen::Matrix<double,6,1> err;

    for (int it = 0; it < IT_MAX; ++it) {
      pinocchio::forwardKinematics(model, local_data, qvec);
      const pinocchio::SE3 iMd = local_data.oMi[target_joint_id].actInv(oMdes);
      err = pinocchio::log6(iMd).toVector();
      if (err.norm() < eps) break;

      pinocchio::computeJointJacobian(model, local_data, qvec, target_joint_id, J);
      // 应用耦合与固定
      for (auto [m,s] : coupled) { J.col(m) += J.col(s); J.col(s).setZero(); }
      for (int id : fixed) J.col(id).setZero();

      pinocchio::Data::Matrix6 Jlog; pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;

      pinocchio::Data::Matrix6 JJt = J * J.transpose();
      JJt.diagonal().array() += damp;
      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);

      // 速度层面的耦合/固定
      for (auto [m,s] : coupled) v[s] = v[m];
      for (int id : fixed) v[id] = 0.0;

      qvec = pinocchio::integrate(model, qvec, v * DT);

      // 位置层的裁剪与从动关节多项式关系
      for (int i = 0; i < qvec.size(); ++i)
        qvec[i] = std::max(minq[i], std::min(qvec[i], maxq[i]));

      for (auto [m,s] : coupled) {
        // finger_idx: 0..4 对应多项式系数组
        qvec[s] = ::deg_to_rad(
          evaluatePolynomial(poly_coeff[finger_idx], 3, ::rad_to_deg(static_cast<float>(qvec[m])))
        );
      }
      for (int id : fixed) qvec[id] = 0.0;
      for (int i = 0; i < qvec.size(); ++i)
        qvec[i] = std::max(minq[i], std::min(qvec[i], maxq[i]));
    }
  };

  // 对五指分别求解（顺序执行，避免线程开销）
  std::vector<Eigen::VectorXd> q_per(5, q_ik);
  for (int f = 0; f < 5; ++f) {
    const int jid = model.getJointId(ftip[f]);
    solve_finger_ik(f, target_poses[f], jid, q_per[f]);
  }

  // 合并回整体关节向量
  // 拇指 0..3
  q_ik[0] = q_per[0][0]; q_ik[1] = q_per[0][1]; q_ik[2] = q_per[0][2]; q_ik[3] = q_per[0][3];
  // 食指 4..6
  q_ik[4] = q_per[1][4]; q_ik[5] = q_per[1][5]; q_ik[6] = q_per[1][6];
  // 中指 7..9
  q_ik[7] = q_per[2][7]; q_ik[8] = q_per[2][8]; q_ik[9] = q_per[2][9];
  // 无名指 10..12
  q_ik[10] = q_per[3][10]; q_ik[11] = q_per[3][11]; q_ik[12] = q_per[3][12];
  // 小指 13..15
  q_ik[13] = q_per[4][13]; q_ik[14] = q_per[4][14]; q_ik[15] = q_per[4][15];

  // 输出到响应
  std::array<float,11> j_out{};
  model_q_to_res_angle(q_ik, j_out);
  for (int i = 0; i < 11; ++i) response.j_ang[i] = j_out[i];
}

} // namespace rh6
} // namespace ruiyan

// ===== CAN socket + ryhandlib glue + thread and ROS1 main =====

// Thread control
static pthread_t g_can_thread_id;
static volatile int g_thread_go = 1;

static inline void sleep_us(int us)
{
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

// ryhandlib hooks: only declare they exist; implementation in ryhandlib_port.c
extern "C" s8_t BusWrite(CanMsg_t stuMsg);
extern "C" void MyHoockCallBck(CanMsg_t stuMsg, void* para);
extern "C" void CallBck0(CanMsg_t stuMsg, void* para);

static void BusReadAnduwTickTask()
{
#ifdef __linux__
  {
    int policy; struct sched_param sch{}; pthread_getschedparam(pthread_self(), &policy, &sch);
    (void)policy; (void)sch;
  }
#endif

  while (g_thread_go)
  {
    auto now = std::chrono::steady_clock::now();
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    uwTick = static_cast<s16_t>(ms % 1000);

    struct can_frame frame{};
    if ((sock > 0) && receive_can_message(sock, &frame))
    {
      CanMsg_t rx{};
      rx.ulId  = frame.can_id;
      rx.ucLen = frame.can_dlc;
      std::memcpy(rx.pucDat, frame.data, frame.can_dlc);
      RyCanServoLibRcvMsg(&stuServoCan, rx);
    }

    sleep_us(100);
  }
}

static void* CanRx_and_uwTick_thread(void* /*arg*/)
{
#ifdef __linux__
  struct sched_param sch{}; sch.sched_priority = 80;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0)
  {
    perror("pthread_setschedparam failed (non-RT fallback)");
  }
#endif
  BusReadAnduwTickTask();
  return nullptr;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rh6_ctrl");
  ros::NodeHandle nh("~");

  std::string can_if = "can0";
  nh.param<std::string>("can_interface", can_if, can_if);
  if (argc > 1 && std::strncmp(argv[1], "can", 3) == 0) can_if = argv[1];
  ROS_INFO("Using CAN interface: %s", can_if.c_str());

  if (!open_can_socket(&sock, &addr, &ifr, can_if.c_str()))
  {
    ROS_ERROR("open_can_socket failed on %s", can_if.c_str());
    sock = 0;
  }

  std::memset(&stuServoCan, 0, sizeof(RyCanServoBus_t));
  stuServoCan.usHookNum = 5;
  stuServoCan.pstuHook = (MsgHook_t*)std::malloc(stuServoCan.usHookNum * sizeof(MsgHook_t));
  if (stuServoCan.pstuHook) std::memset(stuServoCan.pstuHook, 0, stuServoCan.usHookNum * sizeof(MsgHook_t));

  stuServoCan.usListenNum = 32;
  stuServoCan.pstuListen = (MsgListen_t*)std::malloc(stuServoCan.usListenNum * sizeof(MsgListen_t));
  if (stuServoCan.pstuListen) std::memset(stuServoCan.pstuListen, 0, stuServoCan.usListenNum * sizeof(MsgListen_t));

  u8_t ret = RyCanServoBusInit(&stuServoCan, BusWrite, (volatile u16_t*)&uwTick, 1000);
  if (ret == 0)
  {
    for (u8_t i = 0; i < MOTOR_NUM; ++i)
    {
      stuListenMsg[i].ulId = SERVO_BACK_ID(i + 1);
      stuListenMsg[i].pucDat[0] = 0xAA;
      AddListen(&stuServoCan, stuListenMsg + i, CallBck0);
    }
    for (u8_t i = 0; i < MOTOR_NUM; ++i)
    {
      stuListenMsg[MOTOR_NUM + i].ulId = SERVO_BACK_ID(i + 1);
      stuListenMsg[MOTOR_NUM + i].pucDat[0] = 0xA0;
      AddListen(&stuServoCan, stuListenMsg + MOTOR_NUM + i, CallBck0);
    }
  }

  sutServoDataW[0].pucDat[0] = 0xAA;
  sutServoDataW[0].stuCmd.usTp = 4095;
  sutServoDataW[0].stuCmd.usTv = 1000;
  sutServoDataW[0].stuCmd.usTc = 80;
  for (int i = 1; i < MOTOR_NUM; ++i) sutServoDataW[i] = sutServoDataW[0];

  g_thread_go = 1;
  if (pthread_create(&g_can_thread_id, nullptr, CanRx_and_uwTick_thread, nullptr) != 0)
  {
    perror("pthread_create failed");
  }
  else
  {
    pthread_detach(g_can_thread_id);
  }

  RyParam_ClearFault(&stuServoCan, 0, 1);

  try
  {
    ruiyan::rh6::rh6_ctrl node(nh, "rh6");
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
  }

  g_thread_go = 0;
  if (sock > 0) close_can_socket(sock);
  if (stuServoCan.pstuHook) { std::free(stuServoCan.pstuHook); stuServoCan.pstuHook = nullptr; }
  if (stuServoCan.pstuListen) { std::free(stuServoCan.pstuListen); stuServoCan.pstuListen = nullptr; }

  return 0;
}