#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/extract_double.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/systems/primitives/linear_system.h"

namespace drake {
namespace systems {

/// A continuous- or discrete-time Linear Time-Varying system with system
/// matrices described by trajectories.
///
/// @tparam T The scalar element type, which must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
///
/// @ingroup primitive_systems
///
template <typename T>
class TrajectoryLinearSystem final : public TimeVaryingLinearSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TrajectoryLinearSystem)

  /// Constructs a PiecewisePolynomialLinearSystem from a LinearTimeVaryingData
  /// structure.
  ///
  /// @param time_period Defines the period of the discrete time system; use
  ///  time_period=0.0 to denote a continuous time system.  @default 0.0
  TrajectoryLinearSystem(const trajectories::Trajectory<double>& A,
                         const trajectories::Trajectory<double>& B,
                         const trajectories::Trajectory<double>& C,
                         const trajectories::Trajectory<double>& D,
                         double time_period = 0.)
      : TimeVaryingLinearSystem<T>(
            SystemTypeTag<TrajectoryLinearSystem>{}, A.rows(),
            B.cols(), C.rows(), time_period),
        A_(A.Clone()),
        B_(B.Clone()),
        C_(C.Clone()),
        D_(D.Clone()) {}

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit TrajectoryLinearSystem(const TrajectoryLinearSystem<U>& other)
      : TrajectoryLinearSystem<T>(*other.A_, *other.B_, *other.C_, *other.D_,
                                  other.time_period()) {
    this->ConfigureDefaultAndRandomStateFrom(other);
  }

  /// @name Implementations of PiecewisePolynomialLinearSystem<T>'s pure virtual
  /// methods.
  /// @{
  MatrixX<T> A(const T& t) const final {
    return A_->value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> B(const T& t) const final {
    return B_->value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> C(const T& t) const final {
    return C_->value(ExtractDoubleOrThrow(t));
  }
  MatrixX<T> D(const T& t) const final {
    return D_->value(ExtractDoubleOrThrow(t));
  }
  /// @}

 private:
  // Allow different specializations to access each other's private data.
  template <typename>
  friend class TrajectoryLinearSystem;

  const std::unique_ptr<trajectories::Trajectory<double>> A_;
  const std::unique_ptr<trajectories::Trajectory<double>> B_;
  const std::unique_ptr<trajectories::Trajectory<double>> C_;
  const std::unique_ptr<trajectories::Trajectory<double>> D_;
};

// Exclude symbolic::Expression from the scalartype conversion of
// TrajectoryLinearSystem.
namespace scalar_conversion {
template <>
struct Traits<TrajectoryLinearSystem> : public NonSymbolicTraits {};
}  // namespace scalar_conversion

}  // namespace systems
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::systems::TrajectoryLinearSystem)
