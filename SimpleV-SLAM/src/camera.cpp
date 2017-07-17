/*************************************************************************
    > File Name: camera.cpp
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: 2017年06月03日 星期六 20时54分28秒
 ************************************************************************/

#include "myslam/camera.h"
#include "myslam/config.h"
namespace myslam
{
Camera::Camera()
{/**
  fx_= Config::get<float>("camera.fx");
  fy_= Config::get<float>("camera.fy");
  cx_= Config::get<float>("camera.cx");
  cy_= Config::get<float>("camera.cy");
  depth_scale_= Config::get<float>("camera.depth_scale");
*/
}

// Eigen::Vector3d = Eigen::Matrix(double,3,1)
Vector3d Camera::world2camera(const Vector3d& p_w,const SE3& T_cw)
{
  return T_cw * p_w;
}

Vector3d Camera::camera2world(const Vector3d& p_c,const SE3& T_cw)
{
  return T_cw.inverse() * p_c;
}
/**
 * u = fx * X/Z + cx
 * v = fy * Y/Z + cy
 */
Vector2d Camera::camera2pixel(const Vector3d& p_c)
{
  return Vector2d(
    fx_ * (p_c(0,0) / p_c(2,0)) + cx_,
    fy_ * (p_c(1,0) / p_c(2,0)) + cy_  );
}

Vector3d Camera::pixel2camera(const Vector2d& p_p, double depth)
{
  return Vector3d(
    (p_p(0,0)-cx_) * depth /fx_,
    (p_p(1,0)-cy_) * depth /fy_,
    depth );
}
  
Vector2d Camera::world2pixel(const Vector3d& p_w, const SE3& T_cw )
{
  return camera2pixel(world2camera(p_w, T_cw));
}

Vector3d Camera::pixel2world(const Vector2d& p_p, const SE3& T_cw, double depth)
{
  return camera2world (pixel2camera(p_p, depth),T_cw);
}

}
