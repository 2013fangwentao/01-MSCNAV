/*
** navlog.h for MSCNAV in /media/fwt/学习/程序/myprogram/01-MSCNAV/include
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Mon Dec 17 下午3:13:11 2018 little fang
** Last update Tue Dec 17 下午4:33:19 2018 little fang
*/

#ifndef NAVLOG_H_
#define NAVLOG_H_
#include <string>
#define GLOGOUTPUT

#ifdef GLOGOUTPUT
#include "glog/logging.h"
#endif // GLOGOUTPUT
namespace MSCNAV
{

namespace NAVLOG
{
inline void NAVLOGINIT(const char* argv,const std::string& path)
{
#ifdef GLOGOUTPUT
    google::InitGoogleLogging((const char *)argv);
    google::SetLogDestination(google::GLOG_INFO, path.c_str());
#endif
}

template <typename T>
inline void NAVLOG(const T &_t, const std::string &info_level)
{
#ifdef GLOGOUTPUT

    if (info_level == "WARNING" || info_level == "warning")
    {
        LOG(WARNING) << _t;
    }
    else if (info_level == "ERROR" || info_level == "error")
    {
        LOG(ERROR) << _t;
    }
    else if (info_level == "FATAL" || info_level == "fatal")
    {
        LOG(FATAL) << _t;
    }
    else
    {
        LOG(INFO) << _t;
    }
#endif
}

template <typename T>
inline void NAVLOGIF(const T &_t, const std::string &info_level, bool is_log)
{
#ifdef GLOGOUTPUT
    if (info_level == "WARNING" || info_level == "warning")
    {
        LOG_IF(WARNING, is_log) << _t;
    }
    else if (info_level == "ERROR" || info_level == "error")
    {
        LOG_IF(ERROR, is_log) << _t;
    }
    else if (info_level == "FATAL" || info_level == "fatal")
    {
        LOG_IF(FATAL, is_log) << _t;
    }
    else
    {
        LOG_IF(INFO, is_log) << _t;
    }
#endif
}
} // namespace NAVLOG

} // namespace MSCNAV

#endif /* !NAVLOG_H_ */
