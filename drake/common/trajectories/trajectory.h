#pragma once

#include <Eigen/Core>
#include "drake/common/eigen_types.h"

namespace drake {

/**
 * A Trajectory represents a time-varying matrix of doubles. 
 */
class Trajectory {
 public:
  virtual ~Trajectory() {}

  /**
   * Evaluates the trajectory at the given time \p t.
   * @param t The time at which to evaluate the trajectory.
   * @return The matrix of evaluated values.
   */
  virtual drake::MatrixX<double> value(double t) const = 0;

  /**
   * Evaluates a derivative of the Trajectory at the given time \p t.
   *
   * @param t The time at which to evaluate.
   * @param derivative_order If zero, return the value of the
   * Trajectory. Otherwise, take the derivative this many times before
   * evaluating.
   * @return The matrix of evaluated values.
   */
  virtual drake::MatrixX<double> derivative(double t,
                                            int derivative_order = 1) const = 0;

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
