#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace cv;

// Function for triangulation to compute the world cordinates;
void triangulation(
  const vector<KeyPoint>& keypoints_1,
  const vector<KeyPoint>& keypoints_2,
  const std::vector<DMatch>& matches,
  const Mat& R,const Mat& t,
  vector<Point3d>& points
);

// Function for feature Matches;
void find_feature_matches(
   const Mat& img_1, const Mat& img_2,
   std::vector<KeyPoint>& keypoints_1,
   std::vector<KeyPoint>& keypoints_2,
   std::vector<DMatch>& matches
    );

// Function for pose estimation
void pose_estimation(
   std::vector<KeyPoint>& keypoints_1,
   std::vector<KeyPoint>& keypoints_2,
   std::vector<DMatch> matches,
   Mat& R,Mat& t
);

// pixel cordinates converted to camera normalized cordinates;
Point2d pixel2cam (const Point2d& p,const Mat& K );

// Function for Bundle Adjustment;
void bundleAdjustment(
   const vector<Point3f> points_3d,
   const vector<Point2f> points_2d,
   const Mat& K,
   Mat& R,Mat& t
);

int main(int argc,char** argv)
{
   if(argc!=5)
      cerr<<"input pose_estimation_3d2d img_1 img_2 depth_1 depth_2"<<endl;
   Mat img_1 = imread(argv[1],CV_LOAD_IMAGE_COLOR);
   Mat img_2 = imread(argv[2],CV_LOAD_IMAGE_COLOR);
   
   vector<KeyPoint> keypoints_1,keypoints_2;
   vector<DMatch> matches;
   find_feature_matches(img_1,img_2,keypoints_1,keypoints_2,matches);
   cout<<"The Size of Matches() is "<<matches.size()<<endl;
   
   //Build 3D points
   // depth image is an 16 bit unsigned,single channel image
   Mat d1 = imread(argv[3],CV_LOAD_IMAGE_UNCHANGED);
   Mat K = (Mat_<double> (3,3)<<520.9,0,325.1,0,521.0,249.7,0,0,1);
   vector<Point3f> pts_3d;
   vector<Point2f> pts_2d;
   for(DMatch m:matches)
   {
      ushort d = d1.ptr<unsigned short> (int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
      if (d==0)
	 continue;
      float dd = d/10000.0;
      Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt,K);
      pts_3d.push_back(Point3f(p1.x*dd,p1.y*dd,dd));
      pts_2d.push_back(keypoints_2[m.trainIdx].pt);
   }
   
   cout<<"3d-2d pairs: "<<pts_3d.size()<<endl;
   Mat r_3d,t_3d;
   
   solvePnP(pts_3d,pts_2d,K,Mat(),r_3d,t_3d,false);
   Mat R_3d;
   
   //r_3d is the rotation vector,R_3d is the rotation matrix
   cv::Rodrigues(r_3d,R_3d);
   
   cout<<"R_3d = "<<endl<<R_3d<<endl<<"t_3d = "<<endl<<t_3d<<endl;
   cout<<"Calling Bundele Adjustment"<<endl;
   
   bundleAdjustment(pts_3d,pts_2d,K,R_3d,t_3d);
   
   //
   Mat R,t;
   pose_estimation(keypoints_1,keypoints_2,matches,R,t);

   // verify E=t^R*Scalar
   Mat t_x = (Mat_<double> (3,3) <<
      0,	-t.at<double> (2,0),	t.at<double> (1,0),
      t.at<double>(2,0),	0,	-t.at<double> (0,0),
      -t.at<double>(1,0),	t.at<double>(0,0),	0  );
   cout<<endl<<"t^R="<<endl<<t_x*R<<endl;

   //verify the Epipolar constraint;
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

void bundleAdjustment(   const vector<Point3f> points_3d, const vector<Point2f> points_2d,const Mat& K, Mat& R,Mat& t)
{
   //Initialization the g2o;
   typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3>> Block;// dimension of Pose is 6,landmark is 3;
   //Solver for linear equation;
   Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
   //Solver for matrix block;
   Block* solver_ptr = new Block(linearSolver);
   g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
   g2o::SparseOptimizer optimizer;
   optimizer.setAlgorithm(solver);
   
   // vertex
   // For pose of frame
   // VertexSE3Expmap :pose of SE3;
   g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
   Eigen::Matrix3d R_mat;
   R_mat <<
	    R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),
	    R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),
	    R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2);
   pose->setId(0);
   pose->setEstimate(g2o::SE3Quat(R_mat,Eigen::Vector3d(t.at<double> (0,0),t.at<double>(1,0),t.at<double>(2,0))));
   //add the pose of second frame as the optimize variables
   optimizer.addVertex(pose);
   
   int index = 1;
   // For landmark
   for(const Point3f p:points_3d)
   {
      g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
      point->setId(index++);
      point->setEstimate(Eigen::Vector3d(p.x,p.y,p.z));
      point->setMarginalized(true);
      optimizer.addVertex(point);
   }
   g2o::CameraParameters* camera = new g2o::CameraParameters(
      K.at<double>(0,0),Eigen::Vector2d(K.at<double>(0,2),K.at<double>(1,2)),0 );
   camera->setId(0);
   optimizer.addParameter(camera);
   
   //For edge;
   index = 1;
   for (const Point2f p:points_2d)
   {
      g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
      edge->setId(index);
      edge->setVertex(0,dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)));
      edge->setVertex(1,pose);
      edge->setMeasurement(Eigen::Vector2d(p.x,p.y));
      edge->setParameterId(0,0);
      edge->setInformation(Eigen::Matrix2d::Identity());
      optimizer.addEdge(edge);
      index++;
   }
   
   // calculate the optimization time
   chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
   optimizer.setVerbose(true);
   optimizer.initializeOptimization();
   optimizer.optimize(100);
   chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
   chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
   cout<<"Optimization costs time :"<<time_used.count()<<" seconds."<<endl;
   
   cout<<endl<<"after optimization:"<<endl;
   cout<<"T = "<<endl<<Eigen::Isometry3d(pose->estimate()).matrix()<<endl;
}

