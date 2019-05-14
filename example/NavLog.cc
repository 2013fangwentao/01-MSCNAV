/*
** NavLog.cc for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/example
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午3:30:10 2018 little fang
** Last update Wed May 14 下午6:53:50 2019 little fang
*/
#include "util/navlog.hpp"
#include "util/navtime.h"
#include <Eigen/Core>

using namespace mscnav::utiltool;

int main(int argc, char const *argv[])
{

  Eigen::Matrix<double, 3, 3> mat;
  mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  NAVLOGINIT(argv[0], "./log/");
  NAVINFOLOG("%02d \t INFO", 1);
  NAVWARNINGLOG("%02d \t WARNING", 2);
  NAVERRORLOG("%02d \t ERROR", 3);
  return 0;
}
