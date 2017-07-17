/*************************************************************************
    > File Name: frame.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 04:30:00 PM CST
 ************************************************************************/

#ifndef FRAME_H
#define FRAME_H

#include "myslam/common_include.h"
#include "myslam/camera.h"

namespace myslam
{

  class MapPoint;
  class Frame 
  {
  public:
    typedef std::shared_ptr<Frame> Ptr;
    unsigned long 		   id_;
    double			         time_stamp_;
    SE3				   T_cw_;
    Camera::Ptr			   camera_;
    Mat				   color_, depth_;

  public:
    Frame();
    Frame(long id, double time_stamp=0, SE3 T_cw = SE3(),
	    Camera::Ptr camera = nullptr, Mat color = Mat(), Mat depth = Mat());
    ~Frame();
    
    static Frame::Ptr createFrame();
    double findDepth(const cv::KeyPoint& kp);
    Vector3d getCamCenter() const;
        
    bool isInFrame(const Vector3d& pt_world);
  };
}
#endif //FRAME_H
