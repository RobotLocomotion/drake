#pragma once

#include <Eigen/Core>
#include "drake/common/drake_export.h"

namespace drake {

/**
 * A Trajectory represents a path from a start state to a goal state.
 * Each point in the path of the trajectory is represented by a state (a matrix
 * of doubles).
 */
class DRAKE_EXPORT Trajectory {
 public:
  virtual ~Trajectory() {}

  /**
   * Evaluates the trajectory at the given time \p t.
   * @param t The time at which to evaluate the trajectory.
   * @return The output matrix.
   */
  virtual Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(
      double t) const = 0;

  /**
   * @return The length of the output vector. If the output is a matrix,
   * length() is the number of rows in the matrix.
  */
  virtual Eigen::Index length() const = 0;

};

}  // namespace drake
