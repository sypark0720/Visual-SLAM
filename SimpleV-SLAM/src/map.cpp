/*************************************************************************
    > File Name: mappoint.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 05:58:00 PM CST
 ************************************************************************/

#include "myslam/map.h"
namespace myslam
{
  void Map::insertKeyFrame(Frame::Ptr frame)
  {
    cout<<"Key frame size = "<<keyframes_.size()<<endl;
    if (keyframes_.find(frame->id_) == keyframes_.end())
    {
      keyframes_.insert(make_pair(frame->id_,frame));
    }else{
      keyframes_[frame->id_] = frame;
    }
  }
  
  void Map::insertMapPoint (MapPoint::Ptr map_point)
  {
    if(map_points_.find(map_point->id_) == map_points_.end())
    {
      map_points_.insert(make_pair(map_point->id_,map_point));
    } else {
      map_points_[map_point->id_] = map_point;
    }
  }
}
