#pragma once

#include <memory>

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
   *
   * @return A deep copy of this Trajectory.
   */
  virtual std::unique_ptr<Trajectory> Clone() const = 0;

  /**
   * Evaluates the trajectory at the given time \p t.
   * @param t The time at which to evaluate the trajectory.
   * @return The matrix of evaluated values.
   */
  virtual drake::MatrixX<double> value(double t) const = 0;

  /**
   * Takes the derivative of this Trajectory.
   * @param derivative_order The number of times to take the derivative before
   * returning.
   * @return The nth derivative of this object.
   */
  virtual std::unique_ptr<Trajectory> derivative(
      int derivative_order = 1) const = 0;

  /**
   * @return The number of rows in the matrix returned by value().
   */
  virtual Eigen::Index rows() const = 0;

  /**
   * @return The number of columns in the matrix returned by value().
   */
  virtual Eigen::Index cols() const = 0;

  virtual double get_start_time() const = 0;
  virtual double get_end_time() const = 0;
};

}  // namespace drake
