#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/primitives/time_varying_data.h"

namespace drake {
namespace systems {

/// A continuous- or discrete-time Affine Time-Varying system described by a
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
class PiecewisePolynomialAffineSystem final
    : public TimeVaryingAffineSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PiecewisePolynomialAffineSystem)

  /// Constructs a PiecewisePolynomialAffineSystem from a TimeVaryingData
  /// structure.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  PiecewisePolynomialAffineSystem(const TimeVaryingData& data,
                                  double time_period = 0.)
      : PiecewisePolynomialAffineSystem<T>(
            SystemTypeTag<systems::PiecewisePolynomialAffineSystem>{}, data,
            time_period) {}

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit PiecewisePolynomialAffineSystem(
      const PiecewisePolynomialAffineSystem<U>& other)
      : PiecewisePolynomialAffineSystem<T>(other.data_, other.time_period()) {}

  /// @name Implementations of PiecewisePolynomialAffineSystem<T>'s pure virtual
  /// methods.
  /// @{
  MatrixX<T> A(const T& t) const final {
    return data_.A.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> B(const T& t) const final {
    return data_.B.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> f0(const T& t) const final {
    return data_.f0.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> C(const T& t) const final {
    return data_.C.value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> D(const T& t) const final {
    return data_.D.value(ExtractDoubleOrThrow(t));
  }
  VectorX<T> y0(const T& t) const final {
    return data_.y0.value(ExtractDoubleOrThrow(t));
  }
  /// @}

 protected:
  /// Constructor that specifies scalar-type conversion support.
  /// @param converter scalar-type conversion support helper (i.e., AutoDiff,
  /// etc.); pass a default-constructed object if such support is not desired.
  /// See @ref system_scalar_conversion and examples related to scalar-type
  /// conversion support for more details.
  PiecewisePolynomialAffineSystem(SystemScalarConverter converter,
                                  const TimeVaryingData& data,
                                  double time_period)
      : TimeVaryingAffineSystem<T>(std::move(converter), data.A.rows(),
                                   data.B.cols(), data.C.rows(), time_period),
        data_(data) {}

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class PiecewisePolynomialAffineSystem;

  const TimeVaryingData data_;
};

// Exclude symbolic::Expression from the scalartype conversion of
// PiecewisePolynomialAffineSystem.
namespace scalar_conversion {
template <>
struct Traits<PiecewisePolynomialAffineSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake
