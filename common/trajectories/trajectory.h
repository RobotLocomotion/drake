#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"

namespace drake {
namespace trajectories {

/**
 * A Trajectory represents a time-varying matrix, indexed by a single scalar
 * time.
 *
 * @tparam T is any scalar type. This is a header-only abstract class (so no
 * concrete scalar type instantiations need to be provided).
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
  virtual MatrixX<T> value(const T& t) const = 0;

  /**
  * If cols()==1, then evaluates the trajectory at each time @p t, and returns
  * the results as a Matrix with the ith column corresponding to the ith time.
  * Otherwise, if rows()==1, then evaluates the trajectory at each time @p t,
  * and returns the results as a Matrix with the ith row corresponding to
  * the ith time.
  * @throws std::runtime_error if both cols and rows are not equal to 1.
  */
  MatrixX<T> vector_values(const std::vector<T>& t) const {
    if (cols() != 1 && rows() != 1) {
      throw std::runtime_error(
          "This method only supports vector-valued trajectories.");
    }
    if (cols() == 1) {
      MatrixX<T> values(rows(), t.size());
      for (int i = 0; i < static_cast<int>(t.size()); i++) {
        values.col(i) = value(t[i]);
      }
      return values;
    }
    MatrixX<T> values(t.size(), cols());
    for (int i = 0; i < static_cast<int>(t.size()); i++) {
      values.row(i) = value(t[i]);
    }
    return values;
  }

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

  virtual T start_time() const = 0;

  virtual T end_time() const = 0;

 protected:
  // Final subclasses are allowed to make copy/move/assign public.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Trajectory)
  Trajectory() = default;
};

}  // namespace trajectories
}  // namespace drake
