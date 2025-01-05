#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace trajectories {

/**
 * A Trajectory represents a time-varying matrix, indexed by a single scalar
 * time.
 *
 * @tparam_default_scalar
 */
template <typename T>
class Trajectory {
 public:
  virtual ~Trajectory();

  /**
   * @return A deep copy of this Trajectory.
   *
   * @warning Support for overriding this as a virtual function is deprecated
   * and will be removed on or after 2025-08-01. Subclasses should override the
   * protected function DoClone(), instead.
   */
  virtual std::unique_ptr<Trajectory<T>> Clone() const;

  /**
   * Evaluates the trajectory at the given time \p t.
   * @param t The time at which to evaluate the trajectory.
   * @return The matrix of evaluated values.
   *
   * @warning Support for overriding this as a virtual function is deprecated
   * and will be removed on or after 2025-08-01. Subclasses should override the
   * protected function do_value(), instead.
   */
  virtual MatrixX<T> value(const T& t) const { return do_value(t); }

  /**
   * If cols()==1, then evaluates the trajectory at each time @p t, and returns
   * the results as a Matrix with the ith column corresponding to the ith time.
   * Otherwise, if rows()==1, then evaluates the trajectory at each time @p t,
   * and returns the results as a Matrix with the ith row corresponding to
   * the ith time.
   * @throws std::exception if both cols and rows are not equal to 1.
   */
  MatrixX<T> vector_values(const std::vector<T>& t) const;

  /**
   * If cols()==1, then evaluates the trajectory at each time @p t, and returns
   * the results as a Matrix with the ith column corresponding to the ith time.
   * Otherwise, if rows()==1, then evaluates the trajectory at each time @p t,
   * and returns the results as a Matrix with the ith row corresponding to
   * the ith time.
   * @throws std::exception if both cols and rows are not equal to 1.
   */
  MatrixX<T> vector_values(const Eigen::Ref<const VectorX<T>>& t) const;

  /**
   * Returns true iff the Trajectory provides and implementation for
   * EvalDerivative() and MakeDerivative().  The derivative need not be
   * continuous, but should return a result for all t for which value(t) returns
   * a result.
   */
  bool has_derivative() const;

  /**
   * Evaluates the derivative of `this` at the given time @p t.
   * Returns the nth derivative, where `n` is the value of @p derivative_order.
   *
   * @throws std::exception if derivative_order is negative.
   */
  MatrixX<T> EvalDerivative(const T& t, int derivative_order = 1) const;

  /**
   * Takes the derivative of this Trajectory.
   * @param derivative_order The number of times to take the derivative before
   * returning.
   * @return The nth derivative of this object.
   * @throws std::exception if derivative_order is negative.
   */
  std::unique_ptr<Trajectory<T>> MakeDerivative(int derivative_order = 1) const;

  /**
   * @return The number of rows in the matrix returned by value().
   *
   * @warning Support for overriding this as a virtual function is deprecated
   * and will be removed on or after 2025-08-01. Subclasses should override the
   * protected function do_rows(), instead.
   */
  virtual Eigen::Index rows() const { return do_rows(); }

  /**
   * @return The number of columns in the matrix returned by value().
   *
   * @warning Support for overriding this as a virtual function is deprecated
   * and will be removed on or after 2025-08-01. Subclasses should override the
   * protected function do_cols(), instead.
   */
  virtual Eigen::Index cols() const { return do_cols(); }

  /**
   * @warning Support for overriding this as a virtual function is deprecated
   * and will be removed on or after 2025-08-01. Subclasses should override the
   * protected function do_start_time(), instead.
   */
  virtual T start_time() const { return do_start_time(); }

  /**
   * @warning Support for overriding this as a virtual function is deprecated
   * and will be removed on or after 2025-08-01. Subclasses should override the
   * protected function do_end_time(), instead.
   */
  virtual T end_time() const { return do_end_time(); }

 protected:
  // Final subclasses are allowed to make copy/move/assign public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Trajectory);
  Trajectory() = default;

  // This will become pure virtual on 2025-08-01.
  virtual std::unique_ptr<Trajectory<T>> DoClone() const;

  // This will become pure virtual on 2025-08-01.
  virtual MatrixX<T> do_value(const T& t) const;

  virtual bool do_has_derivative() const;

  virtual MatrixX<T> DoEvalDerivative(const T& t, int derivative_order) const;

  virtual std::unique_ptr<Trajectory<T>> DoMakeDerivative(
      int derivative_order) const;

  // This will become pure virtual on 2025-08-01.
  virtual Eigen::Index do_rows() const;

  // This will become pure virtual on 2025-08-01.
  virtual Eigen::Index do_cols() const;

  // This will become pure virtual on 2025-08-01.
  virtual T do_start_time() const;

  // This will become pure virtual on 2025-08-01.
  virtual T do_end_time() const;
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::Trajectory);
