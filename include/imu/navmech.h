/*
** mech.h for MSCNAV in /home/fwt/mypro/01-MSCNAV/include/imu
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Wed May 15 下午4:03:11 2019 little fang
** Last update Tue May 20 下午8:02:45 2019 little fang
*/

#ifndef IMU_MECH_H_
#define IMU_MECH_H_

#include "util/navbase.hpp"
#include "util/navlog.hpp"
#include <Eigen/Dense>
#include <vector>

namespace mscnav
{
class NavMech
{
public:
  NavMech(IMUDATATYPE type, int scale_of_gyro = 0, int scale_of_acce = 0)
    : type_(type), scale_of_gyro_(scale_of_gyro), scale_of_acce_(scale_of_acce)
  {
  }
  ~NavMech() {}

public:
  bool MechanicalArrangement(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &pre_nav_info, NAVINFO &curr_nav_info,
                             Eigen::MatrixXd &phi_mat);

private:
  Eigen::Quaterniond MechAttitudeUpdate(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &pre_nav_info);
  Eigen::Vector3d MechVelocityUpdate(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &pre_nav_info);
  Eigen::Vector3d MechPositionUpdate(const NAVINFO &pre_nav_info, double dt);
  Eigen::MatrixXd MechTransferMat(const IMUDATA &pre_imu_data, const IMUDATA &curr_imu_data, const NAVINFO &nav_info);

private:
  IMUDATATYPE type_;
  int scale_of_gyro_ = 0;
  int scale_of_acce_ = 0;
  NAVINFO nav_info_;
};

} // namespace mscnav
#endif /* !IMU_MECH_H_ */
