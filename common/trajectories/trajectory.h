#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace trajectories {

/**
 * A Trajectory represents a time-varying matrix, indexed by a single
 * scalar double time.
 *
 * @tparam T is a Scalar type for the data that is returned.
 */
template <typename T>
class Trajectory {
 public:
  virtual ~Trajectory() = default;

  /**
   * @return A deep copy of this Trajectory.
   */
  virtual std::unique_ptr<Trajectory<T>> Clone() const = 0;

  /**
   * Evaluates the trajectory at the given time \p t.
   * @param t The time at which to evaluate the trajectory.
   * @return The matrix of evaluated values.
   */
  virtual MatrixX<T> value(double t) const = 0;

  /**
   * Takes the derivative of this Trajectory.
   * @param derivative_order The number of times to take the derivative before
   * returning.
   * @return The nth derivative of this object.
   */
  virtual std::unique_ptr<Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const = 0;

  /**
   * @return The number of rows in the matrix returned by value().
   */
  virtual Eigen::Index rows() const = 0;

  /**
   * @return The number of columns in the matrix returned by value().
   */
  virtual Eigen::Index cols() const = 0;

  virtual double start_time() const = 0;

  virtual double end_time() const = 0;

 protected:
  // Final subclasses are allowed to make copy/move/assign public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Trajectory)
  Trajectory() = default;
};

}  // namespace trajectories
}  // namespace drake
