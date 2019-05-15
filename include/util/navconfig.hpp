/*
** navconfig.hpp for config  in /home/fwt/mypro/01-MSCNAV/include/util
**
** Made by little fang
** Login   <fangwentao>
**
** Started on  Tue May 14 下午8:17:51 2019 little fang
** Last update Wed May 14 下午8:19:09 2019 little fang
*/

#ifndef UTIL_CONFIG_H_
#define UTIL_CONFIG_H_

#include "util/navlog.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdio.h>
#include <string>

namespace mscnav
{
namespace utiltool
{
class ConfigInfo
{
public:
  using Ptr = std::shared_ptr<ConfigInfo>;

private:
  ConfigInfo() {}
  ConfigInfo(ConfigInfo &&) = delete;
  ConfigInfo(const ConfigInfo &) = delete;
  ConfigInfo &operator=(ConfigInfo &&) = delete;
  ConfigInfo &operator=(const ConfigInfo &) = delete;
  static Ptr config_info;
  std::map<std::string, std::string> storage;

public:
  ~ConfigInfo() {}

public:
  static Ptr GetInstance();
  bool open(const char *config_file_path);
  template <typename T>
  T get(std::string key)
  {
    transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (storage.count(key) > 0)
    {
      double value = stod(storage[key]);
      return static_cast<T>(value);
    }
    else
    {
      NAVERRORLOG("The key of %s does not exist, return a default value", key.c_str());
      std::cout << "Press enter to continue" << std::endl;
      getchar();
      return T(0x0);
    }
  }
};

template <>
std::string ConfigInfo::get<std::string>(std::string key)
{
  transform(key.begin(), key.end(), key.begin(), ::tolower);
  if (storage.count(key) > 0)
  {
    return (storage[key]);
  }
  else
  {
    NAVERRORLOG("The key of %s does not exist, return a default value", key.c_str());
    std::cout << "Press enter to continue" << std::endl;
    getchar();
    return "";
  }
}
} // namespace utiltool
} // namespace mscnav

#endif /* !CONFIG */
