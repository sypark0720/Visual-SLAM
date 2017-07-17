/*************************************************************************
    > File Name: mappoint.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 05:36:00 PM CST
 ************************************************************************/

#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam
{
  class Map 
  {
  public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;
    unordered_map<unsigned long, Frame::Ptr> keyframes_;
  
    Map() {}
    
    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);
  };
}
#endif //MAP_H
