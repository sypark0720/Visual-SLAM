#include <iostream>
#include <cmath>
#include <time.h>
#include <tuple>
#include <vector>

using namespace std;

struct Point3 {
  double x;
  double y;
  double z;
};

// Hint: Given p1, p2, p3, the plane normal from the 3 points is (p1-p2) X (p1-p3), X is cross product.

// Given the points and the plane(depicted by 3 point indices),
// find the inliers and store the indices into inlier_points.

void FindInlier(const std::vector<Point3>& points, double threshold,
                int idx1, int idx2, int idx3, std::vector<int>& inlier_points) {
      
  // compute normal vector;
  Point3 p1 ;
  p1.x = points[idx1].x-points[idx2].x;
  p1.y = points[idx1].y-points[idx2].y;
  p1.z = points[idx1].z-points[idx2].z;
  Point3 p2;
  p2.x = points[idx1].x-points[idx3].x;
  p2.y = points[idx1].y-points[idx3].y;
  p2.z = points[idx1].z-points[idx3].z;

  Point3 normal_vector;
  normal_vector.x = p1.y*p2.z-p1.z*p2.y;
  normal_vector.y = p1.z*p2.x-p1.x*p2.z;
  normal_vector.z = p1.x*p2.y-p1.y*p2.x;
    
  for(int i=0; i< points.size();i++){
    double pt[3] = {points[i].x-points[idx1].x,
                    points[i].y-points[idx1].y,
                    points[i].z-points[idx1].z};
    double d = (double)(normal_vector.x*pt[0]+normal_vector.y*pt[1]+normal_vector.z*pt[2])
          /sqrt(normal_vector.x*normal_vector.x+normal_vector.y*normal_vector.y+normal_vector.z*normal_vector.z);
    if(abs(d) < threshold)
      inlier_points.push_back(i);
  }
}

void PlaneRANSAC(const std::vector<Point3>& points,
                double threshold, int iteration, std::vector<int>& inlier_points) {
  
  int iterator_num = 0;
  int N = points.size();

  int idx1, idx2,idx3;
  while(iterator_num < iteration){
    iterator_num++;
    inlier_points.clear();
    
    //select 3 different points
    srand((int)time(0));
    idx1 = rand()%N;
    while(1){
      idx2 = rand()%N;
      if(idx2!=idx1)
        break;
    }
    while(1){
      idx3 = rand()%N;
      if(idx3!=idx2 && idx3!=idx1)
        break;
    }

    FindInlier(points, threshold, idx1, idx2, idx3, inlier_points);
    if(inlier_points.size()>0.7*N)
      break;
  }
}

int main() {
  srand(time(0));
  std::vector<Point3> points;
  Point3 p;
  double a = 1, b = 2, c = 3;

  // Generate ax + by + cz = 1;
  for (int i = 0; i < 20; ++i) {
    p.x = (double)rand()/(double)RAND_MAX;
    p.y = (double)rand()/(double)RAND_MAX;
    p.z = (1.0 - a*p.x - b*p.y) / c;
    points.push_back(p);
  }

  // Insert outlier points
  for (int i = 0; i < 5; ++i) {
    p.x = (double)rand()/(double)RAND_MAX;
    p.y = (double)rand()/(double)RAND_MAX;
    p.z = (1.0 - a*p.x - b*p.y) / c + 100.0;
    points.push_back(p);
  }

  std::vector<int> inlier_indices;
  PlaneRANSAC(points, 0.1, 20, inlier_indices);

  for (int i = 0; i < inlier_indices.size(); ++i) {
    cout << inlier_indices[i] << endl;
  }
  return 0;
}
