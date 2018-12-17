/*
** NavTime.cc for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/example
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午12:56:38 2018 little fang
** Last update Tue Dec 17 下午2:59:58 2018 little fang
*/

#include "navtime.h"
#include "navstruct.h"
using namespace MSCNAV;

int main(int argc, char const *argv[])
{
    NavTime time1 = NavTime::NowTime();
    printf("%s\n", time1.Time2String().c_str());
    NavSleep(2000);
    NavTime time2 = NavTime::NowTime();
    printf("%s\n", time2.Time2String().c_str());
    printf("%s\n", time1 <= time2 ? "True" : "FALSE");
    NavTime time3(2018, 12, 1, 0, 0, 0.0);
    time3 += 60.0;
    NavTime time4 = time3 + NavTime::MAXSECONDOFDAY;
    NavExit();
    return 0;
}
