#pragma once

#include <array>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace math {
class BsplineBasis {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(BsplineBasis);
  BsplineBasis(int order, std::vector<double> knots);

  BsplineBasis(int order, int num_control_points);

  int order() const { return order_; }

  int num_control_points() const { return num_control_points_; }

  const std::vector<double>& knots() const { return knots_; }

  const std::vector<drake::trajectories::PiecewisePolynomial<double>>
  polynomials() const {
    return basis_;
  }

  double EvaluateBasisFunction(int index, double evaluation_time) const;

  drake::trajectories::PiecewisePolynomial<double> ConstructBsplineCurve(
      const std::vector<drake::MatrixX<double>>& control_points) const;

  std::vector<int> ComputeActiveControlPointIndices(
      std::array<double, 2> evaluation_time) const;

  std::vector<int> ComputeActiveControlPointIndices(
      double evaluation_time) const;

  bool operator==(const BsplineBasis& other) const;

 private:
  int order_;
  int num_control_points_;
  std::vector<double> knots_;
  std::vector<drake::trajectories::PiecewisePolynomial<double>> basis_;
  // TODO(avalenzu): Replace usage of this member with
  // PiecewisePolynomial<double>::kEpsilonTime. That ought to work, but it was
  // giving me linker errors.
  double kEpsilonTime_{1e-10};
};
}  // namespace math
}  // namespace drake
