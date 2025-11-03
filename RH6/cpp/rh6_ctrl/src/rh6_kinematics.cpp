#define BOOST_MPL_LIMIT_LIST_SIZE 50
#define BOOST_VARIANT_LIMIT_TYPES 50
#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#include <ros/ros.h>
#include <ros/package.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>

class PinocchioRh6
{
public:
  explicit PinocchioRh6(ros::NodeHandle& nh) : nh_(nh)
  {
    ROS_INFO("hello z13 rh6_pinocchio (ROS1)");

    // 加载 URDF 模型（ROS1 用 ros::package::getPath）
    std::string pkg_path = ros::package::getPath("rh6_ctrl");
    if (pkg_path.empty())
    {
      ROS_ERROR("Package 'rh6_ctrl' not found. Please ensure a proper catkin package exists.");
      throw std::runtime_error("rh6_ctrl package not found");
    }
    std::string urdf_path = pkg_path + "/urdf/ruihand15z.urdf";
    ROS_INFO_STREAM("urdf_path: " << urdf_path);

    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);

    for (std::size_t i = 0; i < data_.oMi.size(); ++i)
    {
      std::stringstream ss;
      ss << "Joint " << i << " pose: translation = "
         << data_.oMi[i].translation().transpose()
         << ", rotation =\n" << data_.oMi[i].rotation();
      ROS_INFO_STREAM(ss.str());
    }

    // 初始化关节状态，确保尺寸与模型匹配
    q_.resize(model_.nq);
    q_.setZero();

    // 发布/订阅（ROS1）
    fk_pub_ = nh_.advertise<geometry_msgs::PoseArray>("fk_result", 10);
    ik_sub_ = nh_.subscribe("ik_target", 10, &PinocchioRh6::ikCallback, this);

    // 定时执行正向运动学（ROS1）
    timer_ = nh_.createTimer(ros::Duration(0.05), &PinocchioRh6::fkCallback, this);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher fk_pub_;
  ros::Subscriber ik_sub_;
  ros::Timer timer_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd q_;

  // 正向运动学回调：计算并发布末端执行器位姿
  void fkCallback(const ros::TimerEvent&)
  {
    static double t = 0.0;
    t += 0.05;
    // 关节3做 0~70度正弦变化（弧度）
    q_(2) = 35 * M_PI / 180.0 + (35.0 * M_PI / 180.0) * std::sin(t);

    // 前向运动学
    pinocchio::forwardKinematics(model_, data_, q_);

    for (pinocchio::JointIndex jid = 0; jid < (pinocchio::JointIndex)model_.njoints; ++jid)
    {
      std::ostringstream os;
      os << std::setw(24) << std::left << model_.names[jid] << ": " << std::fixed
         << std::setprecision(2) << data_.oMi[jid].translation().transpose();
      ROS_INFO_STREAM(os.str());
    }

    // 末端关节位姿（按你原先的命名）
    const auto ee1 = data_.oMi[ model_.getJointId("fz15") ];
    const auto ee2 = data_.oMi[ model_.getJointId("fz25") ];
    const auto ee3 = data_.oMi[ model_.getJointId("fz35") ];
    const auto ee4 = data_.oMi[ model_.getJointId("fz45") ];
    const auto ee5 = data_.oMi[ model_.getJointId("fz55") ];

    // 基座到世界的位姿（可按需参数化）
    const double x=0.1, y=0.1, z=0.1, roll=0.0, pitch=0.0, yaw=0.0;
    Eigen::Vector3d translation(x, y, z);
    Eigen::Quaterniond q_base =
      Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d rotation = q_base.toRotationMatrix();
    pinocchio::SE3 base_to_world(rotation, translation);

    // 转到世界坐标
    std::vector<pinocchio::SE3> ee_world = {
      base_to_world * ee1,
      base_to_world * ee2,
      base_to_world * ee3,
      base_to_world * ee4,
      base_to_world * ee5
    };

    // 转成 ROS PoseArray
    geometry_msgs::PoseArray msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.poses.reserve(ee_world.size());
    for (const auto& T : ee_world)
    {
      geometry_msgs::Pose p;
      p.position.x = T.translation().x();
      p.position.y = T.translation().y();
      p.position.z = T.translation().z();
      Eigen::Quaterniond q(T.rotation());
      p.orientation.w = q.w();
      p.orientation.x = q.x();
      p.orientation.y = q.y();
      p.orientation.z = q.z();
      msg.poses.push_back(p);
    }

    fk_pub_.publish(msg);
  }

  // 逆向运动学回调
  void ikCallback(const geometry_msgs::Pose::ConstPtr& msg)
  {
    Eigen::Vector3d target_pos(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Quaterniond target_quat(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    pinocchio::SE3 target_pose(target_quat.toRotationMatrix(), target_pos);

    const double eps = 1e-4;
    const int IT_MAX = 1000;
    const double DT = 1e-1;
    const double damp = 1e-6;

    pinocchio::Data::Matrix6x J(6, model_.nv);
    J.setZero();

    bool success = false;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    Vector6d err;
    Eigen::VectorXd q_ik = q_;
    Eigen::VectorXd v(model_.nv);

    for (int i = 0; i < IT_MAX; ++i)
    {
      pinocchio::forwardKinematics(model_, data_, q_ik);
      const pinocchio::SE3 iMd = data_.oMi[model_.getJointId("fz15")].actInv(target_pose);
      err = pinocchio::log6(iMd).toVector();

      if (err.norm() < eps)
      {
        success = true;
        break;
      }

      pinocchio::computeJointJacobian(model_, data_, q_ik, model_.getJointId("fz15"), J);
      pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(iMd.inverse(), Jlog);
      J = -Jlog * J;

      pinocchio::Data::Matrix6 JJt;
      JJt.noalias() = J * J.transpose();
      JJt.diagonal().array() += damp;

      v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
      // Integrate configuration. Use pinocchio::integrate from joint-configuration.hpp
      // Older Pinocchio headers may not have algorithm/integrate.hpp; joint-configuration.hpp provides integrate.
      pinocchio::integrate(model_, q_ik, v * DT, q_ik);

      if (!(i % 10))
        ROS_INFO("Iteration %d: error = [%f, %f, %f, %f, %f, %f]",
                 i, err(0), err(1), err(2), err(3), err(4), err(5));
    }

    if (success)
    {
      ROS_INFO("IK Convergence achieved!");
      q_ = q_ik;
    }
    else
    {
      ROS_WARN("IK did not converge to the desired precision.");
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rh6_pinocchio");
  ros::NodeHandle nh;

  try
  {
    PinocchioRh6 node(nh);
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }

  return 0;
}