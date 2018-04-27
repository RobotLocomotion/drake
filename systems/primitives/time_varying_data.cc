#include "drake/systems/primitives/time_varying_data.h"

namespace drake {
namespace systems {

// We use out-of-line definitions to rename the arguments without shadowing.

TimeVaryingData::TimeVaryingData(const std::vector<Eigen::MatrixXd>& Ain,
                                 const std::vector<Eigen::MatrixXd>& Bin,
                                 const std::vector<Eigen::MatrixXd>& f0in,
                                 const std::vector<Eigen::MatrixXd>& Cin,
                                 const std::vector<Eigen::MatrixXd>& Din,
                                 const std::vector<Eigen::MatrixXd>& y0in,
                                 double time_period)
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

TimeVaryingData::TimeVaryingData(const PiecewisePolynomial<double>& Ain,
                                 const PiecewisePolynomial<double>& Bin,
                                 const PiecewisePolynomial<double>& f0in,
                                 const PiecewisePolynomial<double>& Cin,
                                 const PiecewisePolynomial<double>& Din,
                                 const PiecewisePolynomial<double>& y0in)
    : A(Ain),
      B(Bin),
      f0(f0in),
      C(Cin),
      D(Din),
      y0(y0in) {
  DRAKE_DEMAND(A.get_number_of_segments() ==
               B.get_number_of_segments());
  DRAKE_DEMAND(A.get_number_of_segments() ==
               C.get_number_of_segments());
  DRAKE_DEMAND(A.get_number_of_segments() ==
               D.get_number_of_segments());
  DRAKE_DEMAND(A.rows() == A.cols());
  DRAKE_DEMAND(B.rows() == A.cols());
  DRAKE_DEMAND(C.cols() == A.cols());
  DRAKE_DEMAND(D.rows() == C.rows());
  DRAKE_DEMAND(D.cols() == B.cols());

  DRAKE_DEMAND(f0.get_number_of_segments() ==
               A.get_number_of_segments());
  DRAKE_DEMAND(f0.rows() == A.cols());
  DRAKE_DEMAND(y0.get_number_of_segments() ==
               A.get_number_of_segments());
  DRAKE_DEMAND(y0.rows() == C.rows());
}

LinearTimeVaryingData::LinearTimeVaryingData(
    const std::vector<Eigen::MatrixXd>& Ain,
    const std::vector<Eigen::MatrixXd>& Bin,
    const std::vector<Eigen::MatrixXd>& Cin,
    const std::vector<Eigen::MatrixXd>& Din, double time_period)
    : TimeVaryingData(
          Ain, Bin, internal::eigen_vector_zeros(Ain.size(), Ain[0].rows()),
          Cin, Din, internal::eigen_vector_zeros(Cin.size(), Cin[0].rows()),
          time_period) {}

LinearTimeVaryingData::LinearTimeVaryingData(
    const PiecewisePolynomial<double>& Ain,
    const PiecewisePolynomial<double>& Bin,
    const PiecewisePolynomial<double>& Cin,
    const PiecewisePolynomial<double>& Din)
    : TimeVaryingData(Ain, Bin, internal::MakeZeroedPiecewisePolynomial(Ain),
                      Cin, Din, internal::MakeZeroedPiecewisePolynomial(Cin)) {}

}  // namespace systems
}  // namespace drake
