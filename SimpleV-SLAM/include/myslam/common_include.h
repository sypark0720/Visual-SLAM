/*************************************************************************
    > File Name: common_include.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: 2017年06月03日 星期六 20时41分56秒
 ************************************************************************/

#ifndef COMMOM_INCLUDE_H
#define COMMOM_INCLUDE_H

// define an common inlcude file to avoid a long include list

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <sophus/se3.h>
// #include <sophus/so3.h>
// using Sophus::SO3;
using Sophus::SE3;

#include <opencv2/core/core.hpp>
using cv::Mat;

#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>

using namespace std;
#endif
