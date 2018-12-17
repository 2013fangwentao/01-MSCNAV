/*
** navstruct.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/inlcude
**
** Made by little fang
** Login   <fangwentao>
** The Basic Struct of Program
**
** Started on  Mon Dec 16 下午3:54:58 2018 little fang
** Last update Mon Dec 16 下午7:05:16 2018 little fang
*/
#ifndef NAVSTRUCT_H_
#define NAVSTRUCT_H_
#include "stdio.h"
#include <string.h>
#include <memory>

namespace MSCNAV
{

struct IMUDATA
{
    double time_of_week_;
    double acce_[3];
    double gyro_[3];
    IMUDATA()
    {
        time_of_week_ = -1.0;
        memset(acce_, 0x0, sizeof(double) * 3);
    }
};

struct GPSPOSVELDATA
{
    double time_of_week_;
    double position_xyz_[3];     /*unit m  */
    double velocity_xyz_[3];     /*unit m/s*/
    double position_xyz_std_[3]; /*unit m  */
    double velocity_xyz_std_[3]; /*unit m/s*/
};

struct ODODATA
{
    double time_of_week_;
    double odo_value_;
}; //bak for ODO

struct BARODATA
{
    double time_of_week_;
    double baro_value_;
}; //bak for barometer


} // namespace MSCNAV

#endif /* !NAVSTRUCT_H_ */
