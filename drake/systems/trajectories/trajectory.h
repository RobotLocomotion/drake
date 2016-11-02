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
   * @return The number of rows in the matrix returned by value().
   */
  virtual Eigen::Index rows() const = 0;

  /**
   * @return The number of columns in the matrix returned by value().
   */
  virtual Eigen::Index cols() const = 0;
};

}  // namespace drake
