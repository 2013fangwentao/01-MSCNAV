/*
** NavLog.cc for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/example
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午3:30:10 2018 little fang
** Last update Tue Dec 17 下午4:33:50 2018 little fang
*/

#include "navlog.h"
#include "navtime.h"
#include "Eigen/Core"

int main(int argc, char const *argv[])
{

    Eigen::Matrix<double,3,3> mat;
    mat<<1,2,3,4,5,6,7,8,9;
    MSCNAV::NAVLOG::NAVLOGINIT(argv[0],"./log/");
    MSCNAV::NAVLOG::NAVLOG("test log1","WARNING");
    MSCNAV::NAVLOG::NAVLOG("test log2","HAHA");
    MSCNAV::NAVLOG::NAVLOG("test log3","INFO");
    MSCNAV::NAVLOG::NAVLOG("test log4","ERROR");
    MSCNAV::NAVLOG::NAVLOG(mat,"INFO");
    DLOG(INFO)<<"TEST";
    LOG(INFO)<<"\n"<<mat;
    return 0;
}


