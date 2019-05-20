/*
** navmech.cc for MSCNAV in /home/fwt/mypro/01-MSCNAV/src/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Thu May 16 下午8:33:16 2019 little fang
** Last update Tue May 20 下午9:07:35 2019 little fang
*/

#include "imu/navmech.h"
#include "util/navconfig.hpp"

namespace mscnav
{

Eigen::Quaterniond NavMech::MechAttitudeUpdate(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &pre_nav_info)
{
  using Eigen::Quaterniond;
  using Eigen::Vector3d;
  double dt = curr_imu_data.time_of_week_ - pre_imu_data.time_of_week_;
  Vector3d thet_k(curr_imu_data.gyro_), thet_k_1(pre_imu_data.gyro_);

  auto curr_phi = thet_k + thet_k_1.cross(thet_k) / 12.0;
  Quaterniond quat_bb = utiltool::Rotation2Quat(curr_phi);

  auto zeta = (utiltool::wien(pre_nav_info.nav_pos_(0)) + utiltool::wenn(pre_nav_info.nav_pos_, pre_nav_info.nav_vel_)) * dt;
  Quaterniond quat_nn = utiltool::Rotation2Quat(zeta);

  Quaterniond qbn_k = quat_nn * (pre_nav_info.nav_quat_ * quat_bb);
  return qbn_k.normalized();
}

Eigen::Vector3d NavMech::MechVelocityUpdate(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &pre_nav_info)
{
  using Eigen::Matrix3d;
  using Eigen::Vector3d;
  auto &pos = pre_nav_info.nav_pos_, &vel = pre_nav_info.nav_vel_;
  double dt = curr_imu_data.time_of_week_ - pre_imu_data.time_of_week_;
  auto wien = utiltool::wien(pos(0));
  auto wenn = utiltool::wenn(pos, vel);
  Vector3d thet_k(curr_imu_data.gyro_), thet_k_1(pre_imu_data.gyro_);
  Vector3d vb_k(curr_imu_data.acce_), vb_k_1(pre_imu_data.acce_);

  Vector3d gn_vec = utiltool::CalculateGravity(pos);
  Vector3d omega_n = wien * 2.0 + wenn;
  Vector3d delta_gcor = (gn_vec - omega_n.cross(vel)) * dt;

  Matrix3d C_ee = Matrix3d::Identity() - (wenn + wien) * 0.5 * dt;

  Vector3d vrot = thet_k.cross(vb_k) * 0.5;
  Vector3d vscul = (thet_k_1.cross(vb_k) + vb_k_1.cross(thet_k)) / 12.0;

  auto delta_ve = C_ee * pre_nav_info.nav_quat_.toRotationMatrix() * (vb_k + vrot + vscul);
  return (pre_nav_info.nav_vel_ + delta_gcor + delta_ve);
}

Eigen::Vector3d NavMech::MechPositionUpdate(const NAVINFO &pre_nav_info, double dt)
{
  Eigen::Vector3d pos;
  pos(2) = pre_nav_info.nav_pos_(2) - 0.5 * (pre_nav_info.nav_vel_(2) + nav_info_.nav_vel_(2)) * dt;
  double aveh = 0.5 * (nav_info_.nav_pos_(2) + pre_nav_info.nav_pos_(2));
  pos(0) = pos(0) + 0.5 * (pre_nav_info.nav_vel_(0) + pre_nav_info.nav_vel_(0)) * dt / (utiltool::rm(pos(0)) + aveh);
  double aveB = 0.5 * (pre_nav_info.nav_pos_(0) + pos(0));
  pos(1) =
    pre_nav_info.nav_pos_(1) + 0.5 * (pre_nav_info.nav_vel_(1) + pre_nav_info.nav_vel_(1)) * dt / ((utiltool::rn(aveB) + aveh) * cos(aveB));
  return pos;
}

/**
 * @brief  获取状态转移矩阵,排列顺序为 0 位置(北东地) 3 速度 6 姿态角误差 9 陀螺领偏 12 加速度计领偏 15 陀螺比例因子 18 加速度计比例因子
 * @note
 * @param  &pre_imu_data:
 * @param  &curr_imu_data:
 * @param  nav_info:
 * @retval
 */
Eigen::MatrixXd NavMech::MechTransferMat(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &nav_info)
{
  using Eigen::Matrix3d;
  using Eigen::MatrixXd;
  using Eigen::Vector3d;

  utiltool::ConfigInfo::Ptr getconfig = utiltool::ConfigInfo::GetInstance();
  getconfig->get<int>("filter_debug_cov_file");

  static int rows = 18 + scale_of_acce_ + scale_of_gyro_, cols = rows;
  static int corr_time_of_gyro_bias = getconfig->get<int>("corr_time_of_gyro_bias");
  static int corr_time_of_acce_bias = getconfig->get<int>("corr_time_of_acce_bias");
  if (scale_of_gyro_ == 3)
    static int corr_time_of_gyro_scale = getconfig->get<int>("corr_time_of_gyro_scale");
  if (scale_of_acce_ == 3)
    static int corr_time_of_acce_scale = getconfig->get<int>("corr_time_of_acce_scale");

  auto &pos = nav_info.nav_pos_;
  auto &vel = nav_info.nav_vel_;
  auto Rm = utiltool::rm(pos(0));
  auto Rn = utiltool::rn(pos(0));
  auto wien = utiltool::wien(pos(0));
  auto wenn = utiltool::wenn(pos, vel);
  auto Cbn = nav_info.nav_quat_.toRotationMatrix();
  Vector3d fb(curr_imu_data.acce_);
  Vector3d wb(curr_imu_data.gyro_);

  MatrixXd F = MatrixXd::Zero(rows, cols);
  Matrix3d temp;

  //位置对应的误差方程
  F.block<3, 3>(0, 0) << -vel(2) / (Rm + pos(2)), 0, pos(0) / (Rm + pos(0)), pos(1) * tan(pos(0)) / (Rm + pos(0)),
    -(vel(0) / (Rn + pos(2)) + vel(0) * tan(pos(0)) / (Rm + pos(0))), vel(1) / (Rn + pos(2)), 0, 0, 0;
  F.block<3, 3>(0, 3) = Matrix3d::Identity();

  //速度对应的误差方程
  F.block<3, 3>(3, 3) = utiltool::askew(2 * wien + wenn) * -1.0;
  F.block<3, 3>(3, 6) = utiltool::askew(Cbn * fb);
  F.block<3, 3>(3, 12) = Cbn;
  if (scale_of_acce_ == 3)
    F.block<3, 3>(3, 18) = Cbn * (fb.diagonal());
  

}
} // namespace mscnav