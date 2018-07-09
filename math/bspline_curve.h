#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/math/bspline_basis.h"

namespace drake {
namespace math {
template <typename T>
class BsplineCurve {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineCurve);

  BsplineCurve(const BsplineBasis& basis,
               const std::vector<drake::MatrixX<T>>& control_points);

  template <typename T_input>
  BsplineCurve(const BsplineBasis& basis,
               const std::vector<drake::MatrixX<T_input>>& control_points);

  const std::vector<drake::MatrixX<T>>& control_points() const {
    return control_points_;
  }

  int rows() const { return control_points()[0].rows(); }

  int cols() const { return control_points()[0].cols(); }

  double start_time() const { return knots().front(); }

  double end_time() const { return knots().back(); }

  int num_control_points() const { return basis_.num_control_points(); }

  drake::MatrixX<T> value(double time) const;

  drake::MatrixX<T> InitialValue() const;

  const std::vector<double>& knots() const { return basis_.knots(); }

  int order() const { return basis_.order(); }

  int degree() const { return order() - 1; }

  const drake::optional<drake::trajectories::PiecewisePolynomial<double>>&
  piecewise_polynomial() const {
    return piecewise_polynomial_;
  }

  const BsplineBasis& basis() const { return basis_; }

  void InsertKnot(const std::vector<double>& time);

  BsplineCurve<T> Derivative() const;

  bool operator==(const BsplineCurve<T>& other) const;

  math::BsplineCurve<T> CopyBlock(int start_row, int start_col, int block_rows,
                                  int block_cols) const;

  /** Creates a math::BsplineCurve consisting of the first
   * `n` elements of `original`.
   * @returns the newly created math::BsplineCurve.
   * @pre original.cols() == 1
   */
  math::BsplineCurve<T> CopyHead(int n) const;

 private:
  void UpdatePiecewisePolynomial();
  BsplineBasis basis_;
  std::vector<drake::MatrixX<T>> control_points_;
  drake::optional<drake::trajectories::PiecewisePolynomial<double>>
      piecewise_polynomial_{};
};
}  // namespace math
}  // namespace drake
