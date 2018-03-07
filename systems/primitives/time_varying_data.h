#pragma once

#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_throw.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"

namespace drake {
namespace systems {

namespace internal {

// Return a vector of the given size, with result[i] == i * step.
inline std::vector<double> vector_iota(int size, double step) {
  DRAKE_DEMAND(step > 0.);
  std::vector<double> result(size);
  for (int i{0}; i < size; ++i) result[i] = i * step;
  return result;
}

// Return std::vector (of the given size) of Eigen vectors of zeros (with the
// given number of rows).
inline std::vector<Eigen::MatrixXd> eigen_vector_zeros(int size, int rows) {
  std::vector<Eigen::MatrixXd> result(size);
  for (int i{0}; i < size; ++i) {
    result[i] = Eigen::MatrixXd::Zero(rows, 1);
  }
  return result;
}

// Return a PiecewisePolynomial<double> of the same number of segments as pp,
// but with an Eigen::VectorXd whose rows equal the number of rows in the
// Eigen::MatrixXd stored in pp.  The vectors are padded with zeros.
inline PiecewisePolynomial<double> MakeZeroedPiecewisePolynomial(
    const PiecewisePolynomial<double>& pp) {
  const double time_period = pp.getEndTime(0) - pp.getStartTime(0);
  return PiecewisePolynomial<double>::FirstOrderHold(
      vector_iota(pp.getNumberOfSegments() + 1, time_period),
      eigen_vector_zeros(pp.getNumberOfSegments() + 1, pp.rows()));
}

}  // namespace internal

/// Stores matrix data necessary to construct an affine time varying system as a
/// piecewise polynomial trajectory.  The trajectory matrices must adhere to the
/// following dimensions:
/// | Matrix  | Num Rows    | Num Columns |
/// |:-------:|:-----------:|:-----------:|
/// | A       | num states  | num states  |
/// | B       | num states  | num inputs  |
/// | C       | num outputs | num states  |
/// | D       | num outputs | num inputs  |
/// | f0      | num states  | 1           |
/// | y0      | num outputs | 1           |
struct TimeVaryingData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(TimeVaryingData)

  /// Default constructor.
  TimeVaryingData() = default;

  /// Fully-parameterized constructor of vector of MatrixXd; these are
  /// ultimately converted into PiecewisePolynomialTrajectories, whose i-th
  /// element corresponds to time = i * time_period.  These vectors are only
  /// well-defined when time_period > 0 (e.g. for discrete-time systems).  The
  /// matrix values are interpolated linearly in between time steps.
  ///
  /// @param time_period The time period from which to construct the time
  /// vector.  time_period must be greater than zero.
  TimeVaryingData(const std::vector<Eigen::MatrixXd>& A,
                  const std::vector<Eigen::MatrixXd>& B,
                  const std::vector<Eigen::MatrixXd>& f0,
                  const std::vector<Eigen::MatrixXd>& C,
                  const std::vector<Eigen::MatrixXd>& D,
                  const std::vector<Eigen::MatrixXd>& y0, double time_period);

  /// Fully-parameterized constructor of PiecewisePolynomials.
  TimeVaryingData(const PiecewisePolynomial<double>& A,
                  const PiecewisePolynomial<double>& B,
                  const PiecewisePolynomial<double>& f0,
                  const PiecewisePolynomial<double>& C,
                  const PiecewisePolynomial<double>& D,
                  const PiecewisePolynomial<double>& y0);

  PiecewisePolynomialTrajectory A{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory B{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory f0{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory C{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory D{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory y0{PiecewisePolynomial<double>()};
};

/// Stores matrix data necessary to construct a linear time varying system as a
/// piecewise polynomial trajectory.  The trajectory matrices must adhere to the
/// following dimensions:
/// | Matrix  | Num Rows    | Num Columns |
/// |:-------:|:-----------:|:-----------:|
/// | A       | num states  | num states  |
/// | B       | num states  | num inputs  |
/// | C       | num outputs | num states  |
/// | D       | num outputs | num inputs  |
struct LinearTimeVaryingData : TimeVaryingData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearTimeVaryingData)

  /// Default constructor.
  LinearTimeVaryingData() = default;

  /// Fully-parameterized constructor of vector of MatrixXd; these are
  /// ultimately converted into PiecewisePolynomialTrajectories, whose i-th
  /// element corresponds to time = i * time_period.  These vectors are only
  /// well-defined when time_period > 0 (e.g. for discrete-time systems).  The
  /// matrix values are interpolated linearly in between time steps.
  ///
  /// @param time_period The time period from which to construct the time
  /// vector.  time_period must be greater than zero.
  LinearTimeVaryingData(const std::vector<Eigen::MatrixXd>& A,
                        const std::vector<Eigen::MatrixXd>& B,
                        const std::vector<Eigen::MatrixXd>& C,
                        const std::vector<Eigen::MatrixXd>& D,
                        double time_period);

  /// Fully-parameterized constructor of PiecewisePolynomials.
  LinearTimeVaryingData(const PiecewisePolynomial<double>& A,
                        const PiecewisePolynomial<double>& B,
                        const PiecewisePolynomial<double>& C,
                        const PiecewisePolynomial<double>& D);
};

}  // namespace systems
}  // namespace drake
