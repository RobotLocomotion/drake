#pragma once

#include <Eigen/Core>
#include "drake/common/drake_export.h"

namespace drake {

/**
 * A Trajectory represents a path from a start state to a goal state.
 * Each point in the path of the trajectory is represented by a state (a vector
 * of doubles).
 */
class DRAKE_EXPORT Trajectory {
 public:
  virtual ~Trajectory() {}

  virtual Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(
      double t) const = 0;
};

}  // namespace drake
