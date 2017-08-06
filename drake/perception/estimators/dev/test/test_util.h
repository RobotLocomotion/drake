#pragma once

#include <algorithm>  // False positive on `min`.
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <bot_core/pointcloud_t.hpp>

#include "drake/common/drake_assert.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace perception {
namespace estimators {

const double kPi = M_PI;

// TODO(eric.cousineau): Move to a proper LCM conversion type.
void PointCloudToLcm(const Eigen::Matrix3Xd& pts_W,
                     bot_core::pointcloud_t* pmessage);

/*
 * Simple interval class.
 */
struct Interval {
  Interval() {}
  Interval(double min_in, double max_in)
      : min(min_in), max(max_in) {
    DRAKE_DEMAND(min <= max);
  }
  double min{};
  double max{};
  inline bool IsInside(double i) const { return i >= min && i <= max; }
  inline double width() const { return max - min; }
};

struct Bounds {
  Bounds() {}
  Bounds(Interval x_in, Interval y_in, Interval z_in)
      : x(x_in), y(y_in), z(z_in) {}
  Interval x;
  Interval y;
  Interval z;
  inline bool IsInside(double xi, double yi, double zi) const {
    return x.IsInside(xi) && y.IsInside(yi) && z.IsInside(zi);
  }
};

struct IntervalIndex {
  int index;
  Interval interval;
};

struct PlaneIndices {
  IntervalIndex a;  // first plane coordinate
  IntervalIndex b;  // second plane coordinate
  IntervalIndex d;  // depth plane coordinate
};

Eigen::Matrix2Xd Generate2DPlane(double space, Interval x, Interval y);

Eigen::Matrix3Xd Generate2DPlane(double space, PlaneIndices is);

Eigen::Matrix3Xd GenerateBoxPointCloud(double space, Bounds box);

class IcpVisualizer {
 public:
  void PublishCloud(const Eigen::Matrix3Xd& points,
                    const std::string& suffix = "RGBD");

  lcm::DrakeLcm& lcm() { return lcm_; }
 private:
  lcm::DrakeLcm lcm_;
};

}  // namespace estimators
}  // namespace perception
}  // namespace drake
