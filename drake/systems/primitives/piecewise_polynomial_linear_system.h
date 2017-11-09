#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/systems/primitives/linear_system.h"
#include "drake/systems/primitives/time_varying_data.h"

namespace drake {
namespace systems {

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
