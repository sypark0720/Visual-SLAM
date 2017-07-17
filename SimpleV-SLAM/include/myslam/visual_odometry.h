/*************************************************************************
    > File Name: include/myslam/visual_odometry.h
    > Author: ll.pan
    > Mail: ll.pan931204@gmail.com 
    > Created Time: Mon 17 Jul 2017 11:29:44 PM CST
 ************************************************************************/

#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
  class VisualOdometry
  {
  public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState{
	INITIALIZING = -1,
	OK = 0,
	LOST
    };
    
    VOState		state_;
    Map::Ptr	map_;
    Frame::Ptr	ref_;
    Frame::Ptr	curr_;
    
    cv::Ptr<cv::ORB>		orb_;
    vector<cv::Point3f>		pts_3d_ref_;
    vector<cv::Point3f>		pts_3d_world_ref_;
    vector<cv::KeyPoint>	keypoints_curr_;
    Mat				descriptors_curr_;
    Mat 				descriptors_ref_;
    vector<cv::DMatch>		feature_matches_;
    
    SE3 T_cr_estimated_;	// The estimated pose of current frame;
    int num_inliers_;		// Number of inlier features in ICP
    int num_lost_;		// Number of lost times;
    
    // Parameters of feature detect and extract
    int num_of_features_;
    double scale_factor_;
    int level_pyramid_;
    float match_ratio_;
    int max_num_lost_;		// Max number of continuous lost times
    int min_inliers_;		// Minimum inliers 
    
    double keyframe_min_rot; 	// minimal rotation of two keyframes;
    double keyframe_min_trans;// minimal translation of two keyframes;
    
  public:
    VisualOdometry();
    ~VisualOdometry();
    
    bool addFrame (Frame::Ptr frame );
    
  protected:
    
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();
    
    void addKeyFrame();
    void addMapPoints();
    bool checkEstimatedPose();
    bool checkKeyFrame();
  };
}

#endif