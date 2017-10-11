#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

/// Stores time-varying matrix data necessary to construct a Linear Time-Varying
/// system.  The trajectory matrices must obey the following dimensions :
/// | Matrix  | Num Rows    | Num Columns |
/// |:-------:|:-----------:|:-----------:|
/// | A       | num states  | num states  |
/// | B       | num states  | num inputs  |
/// | C       | num outputs | num states  |
/// | D       | num outputs | num inputs  |
struct LinearTimeVaryingData {
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(LinearTimeVaryingData)

  /// Default constructor.
  LinearTimeVaryingData() = default;
  /// Fully-parameterized constructor of PiecewisePolynomials.
  LinearTimeVaryingData(const PiecewisePolynomial<double>& Ain,
                        const PiecewisePolynomial<double>& Bin,
                        const PiecewisePolynomial<double>& Cin,
                        const PiecewisePolynomial<double>& Din)
      : LinearTimeVaryingData(PiecewisePolynomialTrajectory(Ain),
                              PiecewisePolynomialTrajectory(Bin),
                              PiecewisePolynomialTrajectory(Cin),
                              PiecewisePolynomialTrajectory(Din)) {}
  /// Fully-parameterized constructor of PiecewisePolynomialTrajectories.
  LinearTimeVaryingData(const PiecewisePolynomialTrajectory& Ain,
                        const PiecewisePolynomialTrajectory& Bin,
                        const PiecewisePolynomialTrajectory& Cin,
                        const PiecewisePolynomialTrajectory& Din)
      : A(Ain), B(Bin), C(Cin), D(Din) {
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
  }

  PiecewisePolynomialTrajectory A{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory B{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory C{PiecewisePolynomial<double>()};
  PiecewisePolynomialTrajectory D{PiecewisePolynomial<double>()};
};

// Return a vector of the given size, with result[i] == i * step.
inline std::vector<double> vector_iota(int size, double step) {
  std::vector<double> result(size);
  for (int i{0}; i < size; ++i) result[i] = i * step;
  return result;
}

/// A continuous- or discrete-time Linear Time-Varying system described by a
/// piecewise polynomial trajectory of system matrices.
///
/// @tparam T The scalar element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
template <typename T>
class PiecewisePolynomialLinearSystem final
    : public TimeVaryingLinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewisePolynomialLinearSystem)

  /// Constructs a PiecewisePolynomialLinearSystem described by a trajectory of
  /// system matrices.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  PiecewisePolynomialLinearSystem(const PiecewisePolynomialTrajectory& A,
                                  const PiecewisePolynomialTrajectory& B,
                                  const PiecewisePolynomialTrajectory& C,
                                  const PiecewisePolynomialTrajectory& D,
                                  double time_period = 0.)
      : PiecewisePolynomialLinearSystem<T>(
            SystemTypeTag<systems::PiecewisePolynomialLinearSystem>{},
            {A, B, C, D}, time_period) {}

  /// Constructs a PiecewisePolynomialLinearSystem described by a
  /// time-sample-indexed vector of system matrices, whose i-th element
  /// corresponds to time = i * time_period.  The matrix values are interpolated
  /// linearly in between time steps.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  PiecewisePolynomialLinearSystem(const std::vector<Eigen::MatrixXd>& A,
                                  const std::vector<Eigen::MatrixXd>& B,
                                  const std::vector<Eigen::MatrixXd>& C,
                                  const std::vector<Eigen::MatrixXd>& D,
                                  double time_period = 0.)
      : PiecewisePolynomialLinearSystem<T>(
            SystemTypeTag<systems::PiecewisePolynomialLinearSystem>{},
            {PiecewisePolynomial<double>::FirstOrderHold(
                vector_iota(A.size(), time_period), A),
             PiecewisePolynomial<double>::FirstOrderHold(
                 vector_iota(B.size(), time_period), B),
             PiecewisePolynomial<double>::FirstOrderHold(
                 vector_iota(C.size(), time_period), C),
             PiecewisePolynomial<double>::FirstOrderHold(
                 vector_iota(D.size(), time_period), D)},
            time_period) {}

  /// Constructs a PiecewisePolynomialLinearSystem from a LinearTimeVaryingData
  /// structure.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  PiecewisePolynomialLinearSystem(const LinearTimeVaryingData& data,
                                  double time_period = 0.)
      : PiecewisePolynomialLinearSystem<T>(
            SystemTypeTag<systems::PiecewisePolynomialLinearSystem>{}, data,
            time_period) {}

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit PiecewisePolynomialLinearSystem(
      const PiecewisePolynomialLinearSystem<U>& other)
      : PiecewisePolynomialLinearSystem<T>(other.data_, other.time_period()) {}

  /// @name Implementations of PiecewisePolynomialLinearSystem<T>'s pure virtual
  /// methods.
  /// @{
  MatrixX<T> A(const T& t) const final {
    return data_.A.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> B(const T& t) const final {
    return data_.B.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> C(const T& t) const final {
    return data_.C.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> D(const T& t) const final {
    return data_.D.value(ExtractDoubleOrThrow(t));
  }
  /// @}

 protected:
  /// Constructor that specifies scalar-type conversion support.
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion and examples related to scalar-type
  /// conversion support for more details.
  PiecewisePolynomialLinearSystem(SystemScalarConverter converter,
                                  const LinearTimeVaryingData& data,
                                  double time_period)
      : TimeVaryingLinearSystem<T>(std::move(converter), data.A.rows(),
                                   data.B.cols(), data.C.rows(), time_period),
        data_(data) {}

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class PiecewisePolynomialLinearSystem;

  // Return a vector of the given size, with result[i] == i * step.
  static std::vector<double> vector_iota(int size, double step) {
    std::vector<double> result(size);
    for (int i{0}; i < size; ++i) result[i] = i * step;
    return result;
  }

  const LinearTimeVaryingData data_;
};

// Exclude symbolic::Expression from the scalartype conversion of
// PiecewisePolynomialLinearSystem.
namespace scalar_conversion {
template <>
struct Traits<PiecewisePolynomialLinearSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake
