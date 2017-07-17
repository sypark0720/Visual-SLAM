/*************************************************************************
    > File Name: config.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 06:28:00 PM CST
 ************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H
#include "myslam/common_include.h"

namespace myslam
{
  class Config
  {
  private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;
    
    // private constructor makes a singleton;
    Config () {}
  public:
    // public destructor function close the file when deconstructing
    ~Config();
    
    // set a new config file
    static void setParameterFile(const std::string& filename);
    
    // access the parameter values
    template<typename T>
    static T get(const std::string& key)
    {
	return T(Config::config_->file_[key]);
    }
  };
}

#endif 


