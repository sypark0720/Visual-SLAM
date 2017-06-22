#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void features_extract(
	vector<string>& image_index, 
	vector<vector<KeyPoint>>& keypoints_all, 
	vector<Mat>& descriptorr_all,
	vector<vector<Vec3b>>& colors_all);

void features_match(Mat& query, Mat& train, vector<DMatch>& matches);

bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask);

void get_matched_points( 
	vector<KeyPoint>& p1, 
	vector<KeyPoint>& p2, 
	vector<DMatch> matches,
	vector<Point2f>& out_p1, 
	vector<Point2f>& out_p2);

void get_matched_colors(
  vector<Vec3b>& c1, vector<Vec3b>& c2, vector<DMatch> matches, 
  vector<Vec3b>& out_c1, vector<Vec3b>& out_c2 );

void reconstruct(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure);

void maskout_points(vector<Point2f>& p1, Mat& mask);

void save_structure(string file_name, vector<Mat>& rotations, vector<Mat>& motions, Mat& structure, vector<Vec3b>& colors);

int main(int argc,char** argv)
{
  string img1 = "0004.png";
  string img2 = "0006.png";
  vector<string> img_names = { img1, img2 };
  vector<vector<KeyPoint>> key_points_for_all;
  vector<Mat> descriptor_for_all;
  vector<vector<Vec3b>> colors_for_all;
  vector<DMatch> matches;

  Mat K(Matx33d(
    2759.48, 0, 1520.69,
    0, 2764.16, 1006.81,
    0, 0, 1));

  extract_features(img_names, key_points_for_all, descriptor_for_all, colors_for_all);
  match_features(descriptor_for_all[0], descriptor_for_all[1], matches);

  vector<Point2f> p1, p2;
  vector<Vec3b> c1, c2;
  Mat R, T;
  Mat mask;	//mask中大于零的点代表匹配点，等于零代表失配点
  get_matched_colors(colors_for_all[0], colors_for_all[1], matches, c1, c2);
  find_transform(K, p1, p2, R, T, mask);

  //三维重建
  Mat structure;	//4行N列的矩阵，每一列代表空间中的一个点（齐次坐标）
  maskout_points(p1, mask);
  maskout_points(p2, mask);
  reconstruct(K, R, T, p1, p2, structure);

  //保存并显示
  vector<Mat> rotations = { Mat::eye(3, 3, CV_64FC1), R };
  vector<Mat> motions = { Mat::zeros(3, 1, CV_64FC1), T };
  maskout_colors(c1, mask);
  save_structure(".\\Viewer\\structure.yml", rotations, motions, structure, c1);
}

void extract_features(
	vector<string>& image_index,
	vector<vector<KeyPoint>>& keypoints_all,
	vector<Mat>& descriptors_all,
	vector<vector<Vec3b>>& colors_all)
{
  keypoints_all.clear();
  descriptors_all.clear();
  
  Ptr<Feature2D> sift = xfeatures2d::SIFT::create(0, 3, 0.04, 10);
  Mat image;
  for (auto it = image_index.begin(); it != image_index.end(); ++it)
  {
    image = imread(*it);
    if (image.empty()) 
	continue;
    
    vector<KeyPoint> keypoints;
    Mat descriptor;
    sift->detectAndCompute(image, noArray(), keypoints, descriptor);

    //ignore the image feature-less image
    if (keypoints.size() <= 10) continue;
    keypoints_all.push_back(keypoints);
    descriptors_all.push_back(descriptor);

    vector<Vec3b> colors(keypoints.size());
    for (int i = 0; i < keypoints.size(); ++i)
    {
	Point2f& p = keypoints[i].pt;
	colors[i] = image.at<Vec3b>(p.y, p.x);
    }
    colors_all.push_back(colors);
  }
}

void match_features(Mat& query, Mat& train, vector<DMatch>& matches)
{
  vector<vector<DMatch>> knn_matches;
  BFMatcher matcher(NORM_L2);
  // knnMatch find several(k) best matches for each query descriptor.
  matcher.knnMatch(query, train, knn_matches, 2);

  // float min_dist = FLT_MAX;
  float min_dist = 10000;
  for (int r = 0; r < knn_matches.size(); ++r)
  {
    // in this case,matches is less reliable;
    if (knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance) continue;
    
    float dist = knn_matches[r][0].distance;
    if (dist < min_dist) 
	min_dist = dist;
  }

  matches.clear();
  for (size_t r = 0; r < knn_matches.size(); ++r)
  {
    // keep the reliable match;
    if (
	knn_matches[r][0].distance > 0.6*knn_matches[r][1].distance || 
	knn_matches[r][0].distance > 5 * max(min_dist, 10.0f))
	continue;
    
    matches.push_back(knn_matches[r][0]);
  }
}

bool find_transform(Mat& K, vector<Point2f>& p1, vector<Point2f>& p2, Mat& R, Mat& T, Mat& mask)
{
  double focal_length = 0.5*(K.at<double>(0) + K.at<double>(4));
  Point2d principle_point(K.at<double>(2), K.at<double>(5));

  // find Essential Matrix ,& use RANSAC
  Mat E = findEssentialMat(p1, p2, focal_length, principle_point, RANSAC, 0.999, 1.0, mask);
  if (E.empty()) 
    return false;

  double feasible_count = countNonZero(mask);
  cout << (int)feasible_count << " -in- " << p1.size() << endl;
  // If outlier is more than 50%, the result is unreliable for RANSAC
  if (feasible_count <= 15 || (feasible_count / p1.size()) < 0.6)
    return false;

  int pass_count = recoverPose(E, p1, p2, R, T, focal_length, principle_point, mask);

  // The number of points in front of the two cameras should be large enough
  if (((double)pass_count) / feasible_count < 0.7)
    return false;

  return true;
}

void get_matched_points( 
	vector<KeyPoint>& p1, 
	vector<KeyPoint>& p2, 
	vector<DMatch> matches,
	vector<Point2f>& out_p1, 
	vector<Point2f>& out_p2)
{
	out_p1.clear();
	out_p2.clear();
	for (int i = 0; i < matches.size(); ++i)
	{
	  out_p1.push_back(p1[matches[i].queryIdx].pt);
	  out_p2.push_back(p2[matches[i].trainIdx].pt);
	}
}

void get_matched_colors(
	vector<Vec3b>& c1,
	vector<Vec3b>& c2,
	vector<DMatch> matches,
	vector<Vec3b>& out_c1,
	vector<Vec3b>& out_c2
	)
{
  out_c1.clear();
  out_c2.clear();
  for (int i = 0; i < matches.size(); ++i)
  {
    out_c1.push_back(c1[matches[i].queryIdx]);
    out_c2.push_back(c2[matches[i].trainIdx]);
  }
}

void reconstruct(Mat& K, Mat& R, Mat& T, vector<Point2f>& p1, vector<Point2f>& p2, Mat& structure)
{
  Mat proj1(3, 4, CV_32FC1);
  Mat proj2(3, 4, CV_32FC1);

  proj1(Range(0, 3), Range(0, 3)) = Mat::eye(3, 3, CV_32FC1);
  proj1.col(3) = Mat::zeros(3, 1, CV_32FC1);

  R.convertTo(proj2(Range(0, 3), Range(0, 3)), CV_32FC1);
  T.convertTo(proj2.col(3), CV_32FC1);
  Mat fK;
  K.convertTo(fK, CV_32FC1);
  proj1 = fK*proj1;
  proj2 = fK*proj2;

  triangulatePoints(proj1, proj2, p1, p2, structure);
}

void maskout_points(vector<Point2f>& p1, Mat& mask)
{
  vector<Point2f> p1_copy = p1;
  p1.clear();

  for (int i = 0; i < mask.rows; ++i)
  {
    if (mask.at<uchar>(i) > 0)
	p1.push_back(p1_copy[i]);
  }
}

void maskout_colors(vector<Vec3b>& p1, Mat& mask)
{
  vector<Vec3b> p1_copy = p1;
  p1.clear();

  for (int i = 0; i < mask.rows; ++i)
  {
    if (mask.at<uchar>(i) > 0)
	p1.push_back(p1_copy[i]);
  }
}

void save_structure(string file_name, vector<Mat>& rotations, vector<Mat>& motions, Mat& structure, vector<Vec3b>& colors)
{
  int n = (int)rotations.size();

  FileStorage fs(file_name, FileStorage::WRITE);
  fs << "Camera Count" << n;
  fs << "Point Count" << structure.cols;

  fs << "Rotations" << "[";
  for (size_t i = 0; i < n; ++i)
  {
    fs << rotations[i];
  }
  fs << "]";

  fs << "Motions" << "[";
  for (size_t i = 0; i < n; ++i)
  {
    fs << motions[i];
  }
  fs << "]";

  fs << "Points" << "[";
  for (size_t i = 0; i < structure.cols; ++i)
  {
    Mat_<float> c = structure.col(i);
    c /= c(3);	//齐次坐标，需要除以最后一个元素才是真正的坐标值
    fs << Point3f(c(0), c(1), c(2));
  }
  fs << "]";

  fs << "Colors" << "[";
  for (size_t i = 0; i < colors.size(); ++i)
  {
    fs << colors[i];
  }
  fs << "]";
  fs.release();
}

