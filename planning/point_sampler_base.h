#pragma once

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace planning {
/**
 * An abstract base class for implementing ways to sample points from the
 * underlying space an approximate convex cover builder wishes to cover.
 */
class PointSamplerBase {
 public:
  /**
   * Sample num_points from the underlying space and return these points as a
   * matrix where each column represents an underlying point.
   * @param num_threads the number of threads used to sample points.
   */
  Eigen::MatrixXd SamplePoints(int num_points);

  virtual ~PointSamplerBase() {}

 protected:
  // We put the copy/move/assignment constructors as protected to avoid copy
  // slicing. The inherited final subclasses should put them in public
  // functions.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PointSamplerBase);
  PointSamplerBase() = default;

 private:
  virtual Eigen::MatrixXd DoSamplePoints(int num_points) = 0;
};

}  // namespace planning
}  // namespace drake
