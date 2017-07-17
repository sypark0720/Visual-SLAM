/*************************************************************************
    > File Name: mappoint.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 05:16:00 PM CST
 ************************************************************************/

#ifndef MAPPOINT_H
#define MAPPOINT_H

namespace myslam
{
  
  class Frame;
  class MapPoint
  {
  public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long			id_;
    Vector3d			pos_;
    Vector3d			norm_;
    Mat				descriptor_;
    int				observed_times_;
    int				correct_times_; // being an inliner in pose estimation
    
    MapPoint();
    MapPoint(long id, Vector3d position, Vector3d norm);
    
    static MapPoint::Ptr createMapPoint();
  };
}
#endif //MAPPOINT_H
