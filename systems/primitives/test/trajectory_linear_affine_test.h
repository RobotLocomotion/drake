#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

namespace drake {
namespace systems {
namespace test {


// Provides sample trajectories (and the corresponding data) to support
// testing TrajectoryAffineSystem and TrajectoryLinearSystem.
struct TestSystemMatrixTrajectories {
  TestSystemMatrixTrajectories() {
    Eigen::Matrix2d A0;
    A0 << 1, 2, 3, 4;
    Eigen::Matrix2d B0;
    B0 << 5, 6, 7, 8;
    Eigen::Vector2d f00;
    f00 << 9, 10;
    Eigen::Matrix2d C0;
    C0 << 11, 12, 13, 14;
    Eigen::Matrix2d D0;
    D0 << 15, 16, 17, 18;
    Eigen::Vector2d y00;
    y00 << 19, 20;
    for (int i{0}; i < static_cast<int>(times.size()); ++i) {
      Avec[i] = A0 + i * Eigen::Matrix2d::Ones();
      Bvec[i] = B0 + i * Eigen::Matrix2d::Ones();
      f0vec[i] = f00 + i * Eigen::Vector2d::Ones();
      Cvec[i] = C0 + i * Eigen::Matrix2d::Ones();
      Dvec[i] = D0 + i * Eigen::Matrix2d::Ones();
      y0vec[i] = y00 + i * Eigen::Vector2d::Ones();
    }

    using trajectories::PiecewisePolynomial;
    A = PiecewisePolynomial<double>::FirstOrderHold(times, Avec);
    B = PiecewisePolynomial<double>::FirstOrderHold(times, Bvec);
    f0 = PiecewisePolynomial<double>::FirstOrderHold(times, f0vec);
    C = PiecewisePolynomial<double>::FirstOrderHold(times, Cvec);
    D = PiecewisePolynomial<double>::FirstOrderHold(times, Dvec);
    y0 = PiecewisePolynomial<double>::FirstOrderHold(times, y0vec);
  }

  const double kDiscreteTimeStep = 0.1;
  const std::vector<double> times{0., kDiscreteTimeStep, 2 * kDiscreteTimeStep};
  std::vector<Eigen::MatrixXd> Avec{times.size()};
  std::vector<Eigen::MatrixXd> Bvec{times.size()};
  std::vector<Eigen::MatrixXd> f0vec{times.size()};
  std::vector<Eigen::MatrixXd> Cvec{times.size()};
  std::vector<Eigen::MatrixXd> Dvec{times.size()};
  std::vector<Eigen::MatrixXd> y0vec{times.size()};

  trajectories::PiecewisePolynomial<double> A{};
  trajectories::PiecewisePolynomial<double> B{};
  trajectories::PiecewisePolynomial<double> f0{};
  trajectories::PiecewisePolynomial<double> C{};
  trajectories::PiecewisePolynomial<double> D{};
  trajectories::PiecewisePolynomial<double> y0{};
};

}  // namespace test
}  // namespace systems
}  // namespace drake
