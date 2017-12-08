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
  TimeVaryingData(const std::vector<Eigen::MatrixXd>& Ain,
                  const std::vector<Eigen::MatrixXd>& Bin,
                  const std::vector<Eigen::MatrixXd>& f0in,
                  const std::vector<Eigen::MatrixXd>& Cin,
                  const std::vector<Eigen::MatrixXd>& Din,
                  const std::vector<Eigen::MatrixXd>& y0in, double time_period)
      : TimeVaryingData(
            PiecewisePolynomial<double>::FirstOrderHold(
                internal::vector_iota(Ain.size(), time_period), Ain),
            PiecewisePolynomial<double>::FirstOrderHold(
                internal::vector_iota(Bin.size(), time_period), Bin),
            PiecewisePolynomial<double>::FirstOrderHold(
                internal::vector_iota(f0in.size(), time_period), f0in),
            PiecewisePolynomial<double>::FirstOrderHold(
                internal::vector_iota(Cin.size(), time_period), Cin),
            PiecewisePolynomial<double>::FirstOrderHold(
                internal::vector_iota(Din.size(), time_period), Din),
            PiecewisePolynomial<double>::FirstOrderHold(
                internal::vector_iota(y0in.size(), time_period), y0in)) {
    DRAKE_DEMAND(time_period > 0.);
  }

  /// Fully-parameterized constructor of PiecewisePolynomials.
  TimeVaryingData(const PiecewisePolynomial<double>& Ain,
                  const PiecewisePolynomial<double>& Bin,
                  const PiecewisePolynomial<double>& f0in,
                  const PiecewisePolynomial<double>& Cin,
                  const PiecewisePolynomial<double>& Din,
                  const PiecewisePolynomial<double>& y0in)
      : A(PiecewisePolynomialTrajectory(Ain)),
        B(PiecewisePolynomialTrajectory(Bin)),
        f0(PiecewisePolynomialTrajectory(f0in)),
        C(PiecewisePolynomialTrajectory(Cin)),
        D(PiecewisePolynomialTrajectory(Din)),
        y0(PiecewisePolynomialTrajectory(y0in)) {
    DRAKE_DEMAND(A.get_piecewise_polynomial().getNumberOfSegments() ==
                 B.get_piecewise_polynomial().getNumberOfSegments());
    DRAKE_DEMAND(A.get_piecewise_polynomial().getNumberOfSegments() ==
                 C.get_piecewise_polynomial().getNumberOfSegments());
    DRAKE_DEMAND(A.get_piecewise_polynomial().getNumberOfSegments() ==
                 D.get_piecewise_polynomial().getNumberOfSegments());
    DRAKE_DEMAND(A.rows() == A.cols());
    DRAKE_DEMAND(B.rows() == A.cols());
    DRAKE_DEMAND(C.cols() == A.cols());
    DRAKE_DEMAND(D.rows() == C.rows());
    DRAKE_DEMAND(D.cols() == B.cols());

    DRAKE_DEMAND(f0.get_piecewise_polynomial().getNumberOfSegments() ==
                 A.get_piecewise_polynomial().getNumberOfSegments());
    DRAKE_DEMAND(f0.rows() == A.cols());
    DRAKE_DEMAND(y0.get_piecewise_polynomial().getNumberOfSegments() ==
                 A.get_piecewise_polynomial().getNumberOfSegments());
    DRAKE_DEMAND(y0.rows() == C.rows());
  }

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
                        double time_period)
      : TimeVaryingData(
            A, B, internal::eigen_vector_zeros(A.size(), A[0].rows()), C, D,
            internal::eigen_vector_zeros(C.size(), C[0].rows()), time_period) {}

  /// Fully-parameterized constructor of PiecewisePolynomials.
  LinearTimeVaryingData(const PiecewisePolynomial<double>& A,
                        const PiecewisePolynomial<double>& B,
                        const PiecewisePolynomial<double>& C,
                        const PiecewisePolynomial<double>& D)
      : TimeVaryingData(A, B, internal::MakeZeroedPiecewisePolynomial(A), C, D,
                        internal::MakeZeroedPiecewisePolynomial(C)) {}
};

}  // namespace systems
}  // namespace drake
