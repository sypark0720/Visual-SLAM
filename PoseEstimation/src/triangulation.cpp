#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void triangulation(
  const vector<KeyPoint>& keypoints_1,
  const vector<KeyPoint>& keypoints_2,
  const std::vector<DMatch>& matches,
  const Mat& R,const Mat& t,
  vector<Point3d>& points
);

void find_feature_matches(
   const Mat& img_1, const Mat& img_2,
   std::vector<KeyPoint>& keypoints_1,
   std::vector<KeyPoint>& keypoints_2,
   std::vector<DMatch>& matches
    );

void pose_estimation(
   std::vector<KeyPoint>& keypoints_1,
   std::vector<KeyPoint>& keypoints_2,
   std::vector<DMatch> matches,
   Mat& R,Mat& t
);

Point2d pixel2cam (const Point2d& p,const Mat& K );

int main(int argc,char** argv)
{
   if(argc!=3)
      cerr<<"input pose_estimation img_1 img_2"<<endl;
   Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
   Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
   vector<KeyPoint> keypoints_1,keypoints_2;
   vector<DMatch> matches;
   find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
   Mat R,t;
   pose_estimation(keypoints_1,keypoints_2,matches,R,t);

   // verify E=t^R*Scalar
   Mat t_x = (Mat_<double> (3,3) <<
      0,	-t.at<double> (2,0),	t.at<double> (1,0),
      t.at<double>(2,0),	0,	-t.at<double> (0,0),
      -t.at<double>(1,0),	t.at<double>(0,0),	0  );
   cout<<endl<<"t^R="<<endl<<t_x*R<<endl;

   //verify the Epipolar constraint;
   Mat K = (Mat_<double> (3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
   for(DMatch m:matches)
   {
      Point2d pt1 = pixel2cam (keypoints_1[m.queryIdx].pt,K);
      Mat y1 = (Mat_<double>(3,1)<<pt1.x,pt1.y,1);
      Point2d pt2 = pixel2cam(keypoints_2[m.trainIdx].pt,K);
      Mat y2 = (Mat_<double>(3,1)<<pt2.x,pt2.y,1);
      Mat d = y2.t()*t_x*R*y1;
      cout<<endl<<"epipolar constraint = "<<d<<endl;
   }
   
   vector<Point3d> points;
   triangulation(keypoints_1,keypoints_2,matches,R,t,points);
   for(int i = 0;i<matches.size();i++)
   {
      // First Image
      Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt,K);
      Point2d pt1_cam_3d(
	 points[i].x/points[i].z,points[i].y/points[i].z);
      cout.precision(6);
      cout<<"point in the first camera frame: "<<pt1_cam<<endl;
      cout<<"point projected from 3D "<<pt1_cam_3d<<",d="<<points[i].z<<endl;
      //Second Image
      Point2f pt2_cam = pixel2cam(keypoints_2[matches[i].trainIdx].pt,K);
      Mat pt2_trans = R*(Mat_<double>(3,1) <<points[i].x,points[i].y,points[i].z)+t;
      pt2_trans /= pt2_trans.at<double>(2,0);
      cout.precision(6);
      cout<<"point in the second camera frame: "<<pt2_cam<<endl;
      cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl<<endl;
   }
   return 0;
}

void find_feature_matches(const Mat& img_1, const Mat& img_2,std::vector<KeyPoint>& keypoints_1,
			  std::vector<KeyPoint>& keypoints_2, std::vector<DMatch>& matches )
{
   Mat descriptors_1,descriptors_2;
   Ptr<FeatureDetector> detector = ORB::create();
   Ptr<DescriptorExtractor> descriptor = ORB::create();
   Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

   detector ->detect(img_1,keypoints_1);
   detector ->detect(img_2,keypoints_2);
   descriptor ->compute(img_1,keypoints_1,descriptors_1);
   descriptor ->compute(img_2,keypoints_2,descriptors_2);

   std::vector<DMatch> match;
   matcher ->match(descriptors_1,descriptors_2,match);

   double min_dist = 10000,max_dist = 0;
   for(int m =0;m<descriptors_1.rows;m++)
   {
      double dist = match[m].distance;
      if(dist > max_dist)
	 max_dist = dist;
      if(dist < min_dist)
	 min_dist = dist;
   }
//   vector<DMatch> matches;
   for(int m=0;m<descriptors_1.rows;m++)
   {
      if(match[m].distance <= max(2*min_dist,30.0))
	 matches.push_back(match[m]);
   }
}

Point2d pixel2cam(const Point2d& p, const Mat& K)
{
   return Point2d
   (
      ((p.x-K.at<double>(0,2))/K.at<double>(0,0)),
      ((p.y-K.at<double>(1,2))/K.at<double>(1,1))
   );
}

void pose_estimation(vector< KeyPoint >& keypoints_1, vector< KeyPoint >& keypoints_2, vector< DMatch > matches, Mat& R, Mat& t)
{
   Mat K = (Mat_<double> (3,3) <<520.9,0,325.1,0,521,249.7,0,0,1);

   vector<Point2f> points1,points2;
   for(int i = 0;i<(int)matches.size();i++)
   {
      points1.push_back(keypoints_1[matches[i].queryIdx].pt);
      points2.push_back(keypoints_2[matches[i].trainIdx].pt);
   }

   Mat fundamental_matrix;
   fundamental_matrix = findFundamentalMat(points1,points2,CV_FM_8POINT);
   cout<<"fundamental_matrix "<<endl<<fundamental_matrix;

   Mat essential_matrix;
   Point2d principal_point(325.1,249.7);
   double focal_length = 521;
   essential_matrix = findEssentialMat(points1,points2,focal_length,principal_point);
   cout<<endl<<"essential_matrix "<<endl<<essential_matrix;

   Mat homography_matrix;
   homography_matrix = findHomography(points1,points2,RANSAC,3);
   cout<<endl<<"homography_matrix "<<endl<<homography_matrix;

   recoverPose(essential_matrix,points1,points2,R,t,focal_length,principal_point);
   cout<<endl<<"R "<<endl<<R;
   cout<<endl<<"t "<<endl<<t;
}

void triangulation( const vector<KeyPoint>& keypoints_1,  const vector<KeyPoint>& keypoints_2,
		    const std::vector<DMatch>& matches,const Mat& R,const Mat& t,vector<Point3d>& points )
{
   // 3x4 projection matrix of the first camera
   Mat T1 = (Mat_<double>(3,4) <<
      1,0,0,0,
      0,1,0,0,
      0,0,1,0);
   // 3x4 projection matrix of the second camera.
   Mat T2 = (Mat_<double>(3,4) <<
      R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
      R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
      R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));
   Mat K = (Mat_<double>(3,3) <<520.9,0,325.1,0,521,249.7,0,0,1);
   vector<Point2d> pts_1,pts_2;
   for(DMatch m:matches)
   {
      pts_1.push_back(pixel2cam(keypoints_1[m.queryIdx].pt,K));
      pts_2.push_back(pixel2cam(keypoints_2[m.trainIdx].pt,K));
   }
   Mat pts_4d;
   cv::triangulatePoints(T1,T2,pts_1,pts_2,pts_4d);
   
   for(int i=0;i<pts_4d.cols;i++)
   {
      Mat x = pts_4d.col(i);
      x /= x.at<float>(3,0);
      Point3d p(
	 x.at<float>(0,0),
	 x.at<float>(1,0),
	 x.at<float>(2,0)
      );
      points.push_back(p);
   }
      
}
























