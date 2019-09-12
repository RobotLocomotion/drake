#pragma once

#include <memory>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_optional.h"
#include "drake/common/eigen_types.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/math/bspline_basis.h"

namespace drake {
namespace math {
template <typename T>
class BsplineTrajectory final : public trajectories::Trajectory<T> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineTrajectory);

  /** Constructs a B-spline trajectory with the given `basis` and
  `control_points`.*/
  BsplineTrajectory(BsplineBasis<double> basis,
                    std::vector<MatrixX<T>> control_points);

  virtual ~BsplineTrajectory() = default;

  // Required methods for trajectories::Trajectory interface.
  std::unique_ptr<trajectories::Trajectory<T>> Clone() const override;

  MatrixX<T> value(double time) const override;

  std::unique_ptr<trajectories::Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override;

  Eigen::Index rows() const override { return control_points()[0].rows(); }

  Eigen::Index cols() const override { return control_points()[0].cols(); }

  double start_time() const override { return knots()[order() - 1]; }

  double end_time() const override { return knots()[num_control_points()]; }

  // Other methods
  int num_control_points() const { return basis_.num_control_points(); }

  const std::vector<MatrixX<T>>& control_points() const {
    return control_points_;
  }

  /** Returns this->value(this->start_time()) */
  MatrixX<T> InitialValue() const;

  /** Returns this->value(this->end_time()) */
  MatrixX<T> FinalValue() const;

  const std::vector<double>& knots() const { return basis_.knots(); }

  int order() const { return basis_.order(); }

  int degree() const { return order() - 1; }

  const BsplineBasis<double>& basis() const { return basis_; }

  /** Adds new knots at the specified `times`. The control points of the
  trajectory are adjusted such that it produces the same value for any valid
  time before and after this method is called.
  @pre this->knots.front() <= t <= this->knots().back() for all t in `times` */
  void InsertKnots(const std::vector<double>& times);

  bool operator==(const BsplineTrajectory<T>& other) const;

  math::BsplineTrajectory<T> CopyWithSelector(
      const std::function<MatrixX<T>(const MatrixX<T>&)>& select) const;

  math::BsplineTrajectory<T> CopyBlock(int start_row, int start_col,
                                       int block_rows, int block_cols) const;

  /** Creates a math::BsplineTrajectory consisting of the first `n` elements of
  `original`.
  @returns the newly created math::BsplineTrajectory.
  @pre original.cols() == 1 */
  math::BsplineTrajectory<T> CopyHead(int n) const;

  double BasisFunctionValue(int index, double time) const;

 private:
  BsplineBasis<double> basis_;
  std::vector<MatrixX<T>> control_points_;
};
}  // namespace math
}  // namespace drake
