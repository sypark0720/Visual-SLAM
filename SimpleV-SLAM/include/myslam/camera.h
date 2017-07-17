/*************************************************************************
    > File Name: camera.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 04:26:00 PM CST
 ************************************************************************/

#ifndef CAMERA_H
#define CAMERA_H
#include "myslam/common_include.h"

namespace myslam
{
  // Pinhole RGB-D Camera model
  class Camera
  {
  public:
    typedef std::shared_ptr<Camera> Ptr;
    // Camera intrinsics
    float fx_, fy_, cx_, cy_, depth_scale_;
    
    Camera();
    Camera(float fx, float fy, float cx, float cy, float depth_scale = 0):
      fx_ (fx), fy_(fy), cx_(cx), cy_(cy), depth_scale_(depth_scale){}
  
    // Coordinate transform: world, camera, pixel
    Vector3d world2camera(const Vector3d& p_w,const SE3& T_cw);
    Vector3d camera2world(const Vector3d& p_c,const SE3& T_cw);
    Vector2d camera2pixel(const Vector3d& p_c); 
    Vector3d pixel2camera(const Vector2d& p_p,double depth =1);
    Vector2d world2pixel(const Vector3d& p_w,const SE3& T_cw);
    Vector3d pixel2world(const Vector2d& p_p,const SE3& T_cw, double depth = 1);
  };
}

#endif 
