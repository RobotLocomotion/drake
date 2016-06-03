#pragma once

#include <vector>
#include <string>
#include <Eigen/Eigen>

class PieceWiseLinearCurve {
 public:
  PieceWiseLinearCurve() {}
  PieceWiseLinearCurve(std::string& fname) {
    set_from_file(fname);
  }
  void set_from_file(const std::string& fname) {
    FILE* file;
    file = fopen(fname.c_str(),"r");
    int matches;
    Eigen::Vector3d point, prev_point;
    double next_dist = 0.0;
    while((matches = fscanf(file,
                            "%lf %lf %lf\n",point.data(),point.data()+1,point.data()+2))!=EOF) {
      if(matches != 3)
        throw std::logic_error(
            "File \""+fname+"\" cannot be parsed.");
      points_.push_back(point);
      if(distances_.size() > 0) {
        next_dist = distances_[distances_.size() - 1] +
            (point - prev_point).norm();
        prev_point = point;
      }
      distances_.push_back(next_dist);
    }
    fclose(file);
  }
  int number_of_points() const { return points_.size(); }

  double ComputeClosestPoint(const Eigen::Vector3d& p,
    Eigen::Vector3d& closest_point, int& index) const {
    double d_min = 1.0e20;
    double distance;
    Eigen::Vector3d q;

    for(int i = 0; i < points_.size(); ++i) {
      Eigen::Vector3d x1 = points_[i];
      Eigen::Vector3d x2 = points_[(i+1)%points_.size()];

      if(!PointToSegmentDistance(x1, x2, p, q, distance) ){
        double d1 = (x1-p).norm();
        double d2 = (x2-p).norm();
        if( d1<d2 ) {
          q = x1;
          distance = d1;
        } else {
          q = x2;
          distance = d2;
        }
      }
      if(distance < d_min) {
        d_min = distance;
        closest_point = q;
        index = i;
      }
    }
    return distances_[index] + (points_[index]-closest_point).norm();
  }

  Eigen::Vector3d ComputePointAt(double s) const {
    for(int i=0;i<distances_.size();++i){
      if(distances_[i] < s && s < distances_[(i+1)%points_.size()]){
        Eigen::Vector3d x1 = points_[i];
        Eigen::Vector3d x2 = points_[(i+1)%points_.size()];
        double w = (s - distances_[i])/(distances_[(i+1)%points_.size()]-distances_[i]);
        return w * x2 + (1.0-w) * x1;
      }
    }
  }
 private:
  static double PointToLineDistance(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& p) {
    Eigen::Vector3d t = x2 - x1; // Direction along the line, from x1 to x2.
    return (p-x1).dot(t)/t.squaredNorm();
  }

  static bool PointToSegmentDistance(const Eigen::Vector3d& x1,
                                     const Eigen::Vector3d& x2,
                                     const Eigen::Vector3d& p,
                                     Eigen::Vector3d& q, double &distance) {
    Eigen::Vector3d t = x2 - x1; // Direction along the line, from x1 to x2.
    double lambda = (p-x1).dot(t)/t.squaredNorm();
    if(0.0<lambda && lambda<1.0) {
      q = lambda * (x2 - x1) + x1;
      distance = (p-q).norm();
      return true;
    }
    return false; // closest point to line not in segment.
  }

  typedef Eigen::aligned_allocator<Eigen::Vector3d> aligned_vector_allocator;
  typedef std::vector<Eigen::Vector3d, aligned_vector_allocator> aligned_vector;
  aligned_vector points_;
  std::vector<double> distances_;
};