#pragma once

// ROS1 headers
#include <ros/ros.h>
#include <ros/package.h>

#include <chrono>
#include <memory>
#include <array>
#include <string>

// Pinocchio
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

// Standard ROS messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Dense>

// Custom ROS1 messages and services
#include "rh6_cmd/Rh6Cmd.h"
#include "rh6_msg/Rh6Msg.h"
#include "rh6_cmd/Rh6fk.h"
#include "rh6_cmd/Rh6ik.h"

namespace ruiyan {
namespace rh6 {

class rh6_ctrl
{
public:
  // Prefer passing an existing NodeHandle for better spinner control
  explicit rh6_ctrl(ros::NodeHandle& nh, const std::string& name);

  // Subscriber callback (ROS1 style)
  void CmdCallback(const rh6_cmd::Rh6Cmd::ConstPtr& msg);

  // Publish state (can be invoked by timer)
  void PubState();
  void UpdataMotor();

  // Services (ROS1 style: return bool)
  bool rh6fk_callback(rh6_cmd::Rh6fk::Request& req, rh6_cmd::Rh6fk::Response& res);
  bool rh6ik_callback(rh6_cmd::Rh6ik::Request& req, rh6_cmd::Rh6ik::Response& res);

  // IK core logic (using ROS1 service request/response types)
  void rh6ik(pinocchio::Model& model,
             pinocchio::Data& data,
             Eigen::VectorXd& q_ik,
             std::string ftip[5],
             const rh6_cmd::Rh6ik::Request& request,
             rh6_cmd::Rh6ik::Response& response);

  // Helpers
  void req_angle_to_model_q(const std::array<float, 11>& req, Eigen::VectorXd& q, int size);
  void model_q_to_res_angle(const Eigen::VectorXd& q, std::array<float, 11>& res);

private:
  // Timer callback
  void onTimer(const ros::TimerEvent&);

private:
  // Pinocchio models and data
  pinocchio::Model model_fk_, model_ik_, model_, model_l_, model_r_;
  pinocchio::Data data_fk_, data_ik_, data_, data_l_, data_r_;
  Eigen::VectorXd q_fk_, q_ik_, q_iik_, q_;

  // URDF and fingertip names
  std::string urdf_path, urdf_filename_l, urdf_filename_r;
  std::string fingertip_l_[5], fingertip_r_[5];
  std::string fingertip[5];
  std::string fingertip_fk[5];
  std::string fingertip_ik[5];

  // Polynomial coefficients and index
  double poly_coeff[5][4];
  int si{0};

  // Message caches (ROS1 types)
  rh6_msg::Rh6Msg rh6msg;
  rh6_cmd::Rh6Cmd rh6cmd;

  // ROS1 node handle and comm objects
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Publisher ryhand_state_publisher_;
  ros::Subscriber ryhand_cmd_subscriber_;
  ros::ServiceServer server_rh6fk_;
  ros::ServiceServer server_rh6ik_;
};

} // namespace rh6
} // namespace ruiyan