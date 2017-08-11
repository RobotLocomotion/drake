#pragma once

#include <algorithm>  // False positive on `min`.
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <bot_core/pointcloud_t.hpp>
#include <gtest/gtest.h>

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

/*
 * Expect that a rotation matrix belong to SO(3):
 *    R ∈ SO(3) =>
 *      Rᵀ R = I (orthonormal)
 *      det(R) = 1 (right-hand rule)
 * @param R Rotation matrix.
 * @param tolerance  The tolerance for determining equivalence for
 * the orthonormal and right-hand rule cases.
 */
::testing::AssertionResult ExpectRotMat(const Eigen::Matrix3d& R,
                                        double tolerance = 0.0);

/*
 * Compare two SE(3) transforms.
 * @param X_expected Expected SE(3) transform.
 * @param X_actual Actual SE(3) transform.
 * @param tolerance The tolerance for determining equivalence for the
 * rotation matrix cases (using ExpectRotMat) and comparing both transforms
 * as matrices.
 */
::testing::AssertionResult CompareTransforms(
    const Eigen::Isometry3d& X_expected, const Eigen::Isometry3d& X_actual,
    double tolerance = 0.0);

/*
 * Compare `R_actual` against `R_expected` to ensure that the axes are
 * aligned, but may have different signs. This is done by checking:
 *   tr(abs(R_expectedᵀ R_actual)) = 3
 * @param R_expected Expected SO(3) rotation.
 * @param R_actual Actual SO(3) rotation.
 * @param tolerance The tolerance for determining equivalence for the
 * rotation matrix cases and the trace.
 */
::testing::AssertionResult CompareRotMatWithoutAxisSign(
    const Eigen::Matrix3d& R_expected, const Eigen::Matrix3d& R_actual,
    double tolerance = 0.0);

/*
 * Compare `X_actual` against `X_expected`, first comparing translation,
 * then rotations using CompareRotMatWithoutAxisSign.
 * @param X_expected Expected SE(3) transform.
 * @param X_actual Actual SE(3) transform.
 * @param tolerance The tolerance for determining equivalence for the
 * rotation matrix sign-agnostic comparison and the translation vector
 * comparison.
 */
::testing::AssertionResult CompareTransformWithoutAxisSign(
    const Eigen::Isometry3d& X_expected, const Eigen::Isometry3d& X_actual,
    double tolerance = 0.0);

/*
 * Visualize point clouds and frames using Drake Visualizer.
 * Use the `show_frames` director script:
 *   drake_visualizer --script
 *       ./drake/multibody/rigid_body_plant/visualization/show_frames.py
 */
class PointCloudVisualizer {
 public:
  void PublishCloud(const Eigen::Matrix3Xd& points,
                    const std::string& suffix = "RGBD");

  void PublishFrames(
      const std::vector<std::pair<std::string, Eigen::Isometry3d>>& frames);

  lcm::DrakeLcm& lcm() { return lcm_; }
 private:
  lcm::DrakeLcm lcm_;
};

}  // namespace estimators
}  // namespace perception
}  // namespace drake
