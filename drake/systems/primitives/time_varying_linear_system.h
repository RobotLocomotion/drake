#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/systems/primitives/affine_system.h"

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
  /// Default constructor.
  LinearTimeVaryingData() = default;
  /// Fully-parameterized constructor.
  LinearTimeVaryingData(const PiecewisePolynomialTrajectory& Ain,
                        const PiecewisePolynomialTrajectory& Bin,
                        const PiecewisePolynomialTrajectory& Cin,
                        const PiecewisePolynomialTrajectory& Din)
      : A(Ain), B(Bin), C(Cin), D(Din) {
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

/// A discrete or continuous linear time-varying system.
///
/// Let `u` denote the input vector, `x` denote the state vector, and
/// `y` denote the output vector.
///
/// If `time_period > 0.0`, the system will have the following discrete-time
/// state update:
///   @f[ x(t+h) = A(t) x(t) + B(t) u(t), @f]
/// where `h` is the time_period.
/// If `time_period == 0.0`, the system will have the following continuous-time
/// state update:
///   @f[ \dot{x} = A(t) x + B(t) u. @f]
///
/// In both cases, the system will have the output:
///   @f[ y = C(t) x + D(t) u, @f]
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
class TimeVaryingLinearSystem : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TimeVaryingLinearSystem)

  /// Constructs an Linear Time-Varying system with a trajectory of system
  /// matrices.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  ///
  /// Subclasses must use the protected constructor, not this one.
  TimeVaryingLinearSystem(const PiecewisePolynomialTrajectory& A,
                          const PiecewisePolynomialTrajectory& B,
                          const PiecewisePolynomialTrajectory& C,
                          const PiecewisePolynomialTrajectory& D,
                          double time_period = 0.)
      : TimeVaryingLinearSystem<T>(
            SystemTypeTag<systems::TimeVaryingLinearSystem>{}, {A, B, C, D},
            time_period) {}

  /// Constructs an Linear Time-Varying system with a trajectory of system
  /// matrix data.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  ///
  /// Subclasses must use the protected constructor, not this one.
  TimeVaryingLinearSystem(const LinearTimeVaryingData& data,
                          double time_period = 0.)
      : TimeVaryingLinearSystem<T>(
            SystemTypeTag<systems::TimeVaryingLinearSystem>{}, data,
            time_period) {}

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit TimeVaryingLinearSystem(const TimeVaryingLinearSystem<U>& other)
      : TimeVaryingLinearSystem<T>(other.data_, other.time_period()) {}

  /// @name Implementations of TimeVaryingAffineSystem<T>'s pure virtual
  /// methods.
  /// @{
  MatrixX<T> A(const T& t) const final {
    return data_.A.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> B(const T& t) const final {
    return data_.B.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> f0(const T&) const final {
    return VectorX<T>::Zero(this->num_states());
  }
  MatrixX<T> C(const T& t) const final {
    return data_.C.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> D(const T& t) const final {
    return data_.D.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> y0(const T&) const final {
    return VectorX<T>::Zero(this->num_outputs());
  }
  /// @}

 protected:
  /// Constructor that specifies scalar-type conversion support.
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion and examples related to scalar-type
  /// conversion support for more details.
  TimeVaryingLinearSystem(SystemScalarConverter converter,
                          const LinearTimeVaryingData& data, double time_period)
      : TimeVaryingAffineSystem<T>(std::move(converter), data.A.rows(),
                                   data.B.cols(), data.C.rows(), time_period),
        data_(data) {}

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class TimeVaryingLinearSystem;

  const LinearTimeVaryingData data_;
};

// Exclude symbolic::Expression from the scalartype conversion of
// TimeVaryingLinearSystem.
namespace scalar_conversion {
template <>
struct Traits<TimeVaryingLinearSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake
