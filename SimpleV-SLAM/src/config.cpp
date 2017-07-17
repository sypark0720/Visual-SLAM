/*************************************************************************
    > File Name: config.cpp
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 06:58:00 PM CST
 ************************************************************************/

#include "myslam/config.h"

// config_ is a pointer which point a Config object, 
// Config is a Class, and file_ is a object of cv::FileStorage
namespace myslam
{
  void Config::setParameterFile(const std::string& filename)
  {
    if (config_ == nullptr)
      config_ = shared_ptr<Config>(new Config);
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false)
    {
      std::cerr << "parameter file " << filename << "doesn't exist " << std::endl;
      config_->file_.release();
      return;
    }
  }
  Config::~Config()
  {
    if(file_.isOpened())
      file_.release();
  }
  
  shared_ptr<Config> Config::config_ = nullptr;
}

