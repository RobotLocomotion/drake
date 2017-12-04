#pragma once

#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace test {

static constexpr double kDiscreteTimeStep = 0.1;

// A helper for accessing the underlying PiecewisePolynomialTrajectory data.
struct MatrixData {
  MatrixData() {}
  /// Fully-parameterized constructor.
  MatrixData(const std::vector<double>& times_in,
             const std::vector<Eigen::MatrixXd>& Avec_in,
             const std::vector<Eigen::MatrixXd>& Bvec_in,
             const std::vector<Eigen::MatrixXd>& f0vec_in,
             const std::vector<Eigen::MatrixXd>& Cvec_in,
             const std::vector<Eigen::MatrixXd>& Dvec_in,
             const std::vector<Eigen::MatrixXd>& y0vec_in)
      : times(times_in),
        Avec(Avec_in),
        Bvec(Bvec_in),
        f0vec(f0vec_in),
        Cvec(Cvec_in),
        Dvec(Dvec_in),
        y0vec(y0vec_in) {}
  std::vector<double> times{};
  std::vector<Eigen::MatrixXd> Avec;
  std::vector<Eigen::MatrixXd> Bvec;
  std::vector<Eigen::MatrixXd> f0vec;
  std::vector<Eigen::MatrixXd> Cvec;
  std::vector<Eigen::MatrixXd> Dvec;
  std::vector<Eigen::MatrixXd> y0vec;
};

// Define a simple TimeVaryingData struct via member assignment.
std::pair<TimeVaryingData, MatrixData> ExampleAffineTimeVaryingData() {
  const std::vector<double> times{0., kDiscreteTimeStep, 2 * kDiscreteTimeStep};
  std::vector<Eigen::MatrixXd> Avec(times.size());
  std::vector<Eigen::MatrixXd> Bvec(times.size());
  std::vector<Eigen::MatrixXd> f0vec(times.size());
  std::vector<Eigen::MatrixXd> Cvec(times.size());
  std::vector<Eigen::MatrixXd> Dvec(times.size());
  std::vector<Eigen::MatrixXd> y0vec(times.size());
  Eigen::Matrix2d A0;
  A0 << 1, 2, 3, 4;
  Eigen::Matrix2d B0;
  B0 << 5, 6, 7, 8;
  Eigen::Vector2d f0;
  f0 << 9, 10;
  Eigen::Matrix2d C0;
  C0 << 11, 12, 13, 14;
  Eigen::Matrix2d D0;
  D0 << 15, 16, 17, 18;
  Eigen::Vector2d y0;
  y0 << 19, 20;
  for (int i{0}; i < static_cast<int>(times.size()); ++i) {
    Avec[i] = A0 + i * Eigen::Matrix2d::Ones();
    Bvec[i] = B0 + i * Eigen::Matrix2d::Ones();
    f0vec[i] = f0 + i * Eigen::Vector2d::Ones();
    Cvec[i] = C0 + i * Eigen::Matrix2d::Ones();
    Dvec[i] = D0 + i * Eigen::Matrix2d::Ones();
    y0vec[i] = y0 + i * Eigen::Vector2d::Ones();
  }

  const TimeVaryingData result(Avec, Bvec, f0vec, Cvec, Dvec, y0vec,
                               kDiscreteTimeStep);

  return std::make_pair(
      result, MatrixData{times, Avec, Bvec, f0vec, Cvec, Dvec, y0vec});
}

}  // namespace test
}  // namespace systems
}  // namespace drake
