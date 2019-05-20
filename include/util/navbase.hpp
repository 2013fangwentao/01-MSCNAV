/*
** navbase.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/inlcude
**
** Made by little fang
** Login   <fangwentao>
** The Basic Struct of Program
**
** Started on  Mon Dec 16 下午3:54:58 2018 little fang
** Last update Tue May 20 下午9:00:21 2019 little fang
*/
#ifndef NAVSTRUCT_H_
#define NAVSTRUCT_H_
#include "glog/logging.h"
#include "stdio.h"
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <memory>
#include <string.h>
#include <thread>

namespace mscnav
{

template <typename _Ty>
inline _Ty pow3(const _Ty &_t)
{
  return _t * _t * _t;
}

template <typename _Ty>
inline _Ty pow2(const _Ty &_t)
{
  return _t * _t;
}

const double PI(3.141592653589793238462643383279);
const double EPSILON(1e-15);
const double e_5(1.0e-5);
const double e_6(1.0e-6);
const double e5(1.0e5);
const double e6(1.0e6);
const double deg2rad(PI / 180.0);
const double rad2deg(180.0 / PI);
const double dh2rs(PI / 180.0 / 3600.0);
const double rs2dh(180.0 / PI * 3600.0);
const double oneH(3600.0);
const double J2(0.00108263);
const double J4(-2.37091222e-6);
const double J6(6.08347e-9);
const double WGS84_AngleRate(7.2921151467E-5);
const double WGS84_GM(3.986005e14);
const double WGS84_a(6378137.0);
const double WGS84_b(6356752.3142);
const double WGS84_e1(sqrt(pow2(WGS84_a) - pow2(WGS84_b)) / WGS84_a);
const double WGS84_e2(sqrt(pow2(WGS84_a) - pow2(WGS84_b)) / WGS84_b);
const double WGS84_f(1 / 298.257223563);

enum IMUDATATYPE
{
  UNKNOWN = 0,
  INCRE = 1,
  RATE = 2
};

/**
 * @brief  IMU raw data
 * @note
 * @retval None
 */
struct IMUDATA
{
  double time_of_week_;
  double acce_[3];
  double gyro_[3];
  IMUDATA()
  {
    time_of_week_ = -1.0;
    memset(acce_, 0x0, sizeof(double) * 3);
    memset(gyro_, 0x0, sizeof(double) * 3);
  }
};

struct GPSDATA
{
  double time_of_week_;
  size_t type_;                /* rtk type fixed or unfixed*/
  double position_xyz_[3];     /*unit m  */
  double velocity_xyz_[3];     /*unit m/s*/
  double position_xyz_std_[3]; /*unit m  */
  double velocity_xyz_std_[3]; /*unit m/s*/
};

struct ODODATA
{
  double time_of_week_;
  double odo_value_;
}; // bak for ODO

struct BARODATA
{
  double time_of_week_;
  double baro_value_;
}; // bak for barometer

struct NAVINFO
{
  Eigen::Vector3d nav_pos_;
  Eigen::Vector3d nav_vel_;
  Eigen::Quaterniond nav_quat_;
  Eigen::Vector3d nav_fn_;
  Eigen::Vector3d nav_wibb_;
  Eigen::Vector3d nav_acce_bias_;
  Eigen::Vector3d nav_gyro_bias_;
  Eigen::Vector3d nav_acce_scale_;
  Eigen::Vector3d nav_gyro_scale_;
  NAVINFO()
  {
    nav_pos_ = Eigen::Vector3d{0, 0, 0};
    nav_vel_ = Eigen::Vector3d{0, 0, 0};
    nav_fn_ = Eigen::Vector3d{0, 0, 0};
    nav_wibb_ = Eigen::Vector3d{0, 0, 0};
    nav_acce_bias_ = Eigen::Vector3d{0, 0, 0};
    nav_gyro_bias_ = Eigen::Vector3d{0, 0, 0};
    nav_acce_scale_ = Eigen::Vector3d{0, 0, 0};
    nav_gyro_scale_ = Eigen::Vector3d{0, 0, 0};
    nav_quat_ = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
  }
};

namespace utiltool
{

using Eigen::Vector3d;

inline double rm(double B) { return WGS84_a * (1 - WGS84_e1 * WGS84_e1) / pow3(sqrt(1 - pow2(WGS84_e1 * sin(B)))); }

inline double rn(double B) { return WGS84_a / sqrt(1 - pow2(WGS84_e1 * sin(B))); }

inline Vector3d wien(double B) { return Vector3d(WGS84_AngleRate * cos(B), 0, -WGS84_AngleRate * sin(B)); }

inline Vector3d wenn(const Vector3d &pos, const Vector3d &vel)
{
  return Vector3d(vel(1) / (rn(pos(0)) + pos(2)), -vel(0) / (rm(pos(0)) + pos(2)), vel(1) * tan(pos(0)) / (rm(pos(0)) + pos(2)));
}

inline void NavSleep(int _milliseconds)
{
  std::chrono::milliseconds msecond(_milliseconds);
  std::this_thread::sleep_for(msecond);
}

inline void NavExit(const std::string &info = "Exitting...")
{
  std::cout << info << std::endl;
  NavSleep(1000);
  exit(0);
}

Eigen::Quaterniond Rotation2Quat(const Eigen::Vector3d &rv)
{
#define F1 (2 * 1) // define: Fk=2^k*k!
#define F2 (F1 * 2 * 2)
#define F3 (F2 * 2 * 3)
#define F4 (F3 * 2 * 4)
#define F5 (F3 * 2 * 5)
  // float n2 = rv.i*rv.i + rv.j*rv.j + rv.k*rv.k, c, f;
  double n2 = rv.squaredNorm(), c, f;

  if (n2 < (M_PI / 180.0 * M_PI / 180.0)) // 0.017^2 默认为0 时特殊考虑
  {
    double n4 = n2 * n2;
    c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);
    f = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
  }
  else
  {
    double n_2 = sqrt(n2) / 2.0;
    c = cos(n_2);
    f = sin(n_2) / n_2 * 0.5;
  }
  return Eigen::Quaterniond(c, f * rv(0), f * rv(1), f * rv(2));
}

Eigen::Vector3d CalculateGravity(const Eigen::Vector3d &pos)
{
  double gn = 9.7803267715 * (1 + 0.0052790414 * sin(pos(0)) * sin(pos(0)) + 0.0000232719 * pow3(sin(pos(0))) * sin(pos(0)));
  gn += (-0.0000030876910891 + 0.0000000043977311 * sin(pos(0)) * sin(pos(0))) * pos(2);
  gn += 0.0000000000007211 * pos(2) * pos(2);
  return Eigen::Vector3d(0, 0, gn);
}

Eigen::Matrix3d askew(Eigen::Vector3d const &v)
{
  Eigen::Matrix3d result;
  result << 0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0;
  return result;
}

Eigen::Matrix3d coeff_of_gn(double Rm, double Rn, const Vector3d &pos)
{
  Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
  mat(2, 2) = (CalculateGravity(pos)(2) / (sqrt(Rm * Rn) + pos(2))) * 2;
  return mat;
}

} // namespace utiltool
} // namespace mscnav

#endif /* !NAVSTRUCT_H_ */
