/*
** navstruct.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/inlcude
**
** Made by little fang
** Login   <fangwentao>
** The Basic Struct of Program
**
** Started on  Mon Dec 16 下午3:54:58 2018 little fang
** Last update Wed May 14 下午7:32:48 2019 little fang
*/
#ifndef NAVSTRUCT_H_
#define NAVSTRUCT_H_
#include "glog/logging.h"
#include "stdio.h"
#include <chrono>
#include <memory>
#include <string.h>
#include <thread>
#include <Eigen/Dense>

namespace mscnav
{

namespace utiltool
{

inline void NavSleep(int _milliseconds)
{
  std::chrono::milliseconds msecond(_milliseconds);
  std::this_thread::sleep_for(msecond);
}

inline void NavExit(const std::string &info = "Exitting...")
{
  printf("%s\n", info.c_str());
  NavSleep(1000);
  exit(0);
}

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

} // namespace utiltool
} // namespace mscnav

#endif /* !NAVSTRUCT_H_ */
