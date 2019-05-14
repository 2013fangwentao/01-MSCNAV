/*
** navlog.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午3:13:11 2018 little fang
** Last update Wed May 14 下午6:53:56 2019 little fang
*/

#ifndef NAVLOG_H_
#define NAVLOG_H_
#include <string>
#define GLOGOUTPUT

#ifdef GLOGOUTPUT
#include "glog/logging.h"
#endif // GLOGOUTPUT
namespace mscnav
{
namespace utiltool
{
  
inline void NAVLOGINIT(const char *argv, const std::string &path)
{
#ifdef GLOGOUTPUT
  google::InitGoogleLogging((const char *)argv);
  google::SetLogDestination(google::GLOG_INFO, path.c_str());
#endif
}

#ifdef GLOGOUTPUT
#define NAVINFOLOG(format, ...)                                                                                                            \
  {                                                                                                                                        \
    char log[4096] = {0};                                                                                                                  \
    sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
    LOG(INFO) << log << std::endl;                                                                                                         \
  }

#define NAVWARNINGLOG(format, ...)                                                                                                         \
  {                                                                                                                                        \
    char log[4096] = {0};                                                                                                                  \
    sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
    LOG(WARNING) << log << std::endl;                                                                                                      \
  }

#define NAVERRORLOG(format, ...)                                                                                                           \
  {                                                                                                                                        \
    char log[4096] = {0};                                                                                                                  \
    sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
    LOG(ERROR) << log << std::endl;                                                                                                        \
  }

#define NAVFATALLOG(format, ...)                                                                                                           \
  {                                                                                                                                        \
    char log[4096] = {0};                                                                                                                  \
    sprintf(log, format, ##__VA_ARGS__);                                                                                                   \
    LOG(FATAL) << log << std::endl;                                                                                                        \
  }
#else
#define NAVINFOLOG(format, ...)
#define NAVWARNINGLOG(format, ...)
#define NAVERRORLOG(format, ...)
#define NAVFATALLOG(format, ...)
#endif

} // namespace untiltool

} // namespace mscnav

#endif /* !NAVLOG_H_ */
