/*
** NavTime.cc for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/example
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午12:56:38 2018 little fang
** Last update Tue Dec 17 下午1:23:58 2018 little fang
*/

#include "navtime.h"
using namespace MSCNAV;

int main(int argc, char const *argv[])
{
    NavTime time1 = NavTime::NowTime();
    printf("%s",time1.Time2String("%04d %03d %.1f",NavTime::DOYTIME).c_str());
    getchar();
    return 0;
}

