#ifndef HD_CONFIG_H
#define HD_CONFIG_H

#include <opencv2/core/core.hpp>
#include <iostream>

namespace hd_utility
{

class Config
{
private:
  Config();

public:
  ~Config();

  static Config& instance();

  static Config* instancePtr();

  static void setParamFile(const std::string& file_name);

  template <typename T>
  static T get(const std::string& key)
  {
    T t;
    Config::instancePtr()->file_[key] >> t;
    return t;
  }

private:
  static Config* single_instance_;

  cv::FileStorage file_;

};

} // namespace hd_utility

#endif //HD_CONFIG_H