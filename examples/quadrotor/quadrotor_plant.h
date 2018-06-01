#pragma once

#include <memory>

#include <Eigen/Core>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace quadrotor {

/// The Quadrotor - an underactuated aerial vehicle. This version of the
/// Quadrotor is implemented to match the dynamics of the plant specified in
/// the `quadrotor.urdf` model file.
template <typename T>
class QuadrotorPlant final : public systems::LeafSystem<T> {
 public:
  QuadrotorPlant();
  QuadrotorPlant(double m_arg, double L_arg, const Eigen::Matrix3d& I_arg,
                 double kF_arg, double kM_arg);

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit QuadrotorPlant(const QuadrotorPlant<U>&);

  ~QuadrotorPlant() override;

  int get_input_size() const { return kInputDimension; }

  int get_num_states() const { return kStateDimension; }

  void set_state(systems::Context<T>* context, const VectorX<T>& x) const {
    context->get_mutable_continuous_state_vector().SetFromVector(x);
  }

  double m() const { return m_; }
  double g() const { return g_; }

 protected:
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  /// Declares that the system has no direct feedthrough from any input to any
  /// output.
  ///
  /// The QuadrotorPlant is incompatible with the symbolic::Expression scalar
  /// type because it invokes the Cholesky LDLT decomposition, which uses
  /// conditionals in its implementation. Therefore, we must specify sparsity
  /// by hand.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 private:
  // Allow different specializations to access each other's private data.
  template <typename> friend class QuadrotorPlant;

  static constexpr int kStateDimension{12};
  static constexpr int kInputDimension{4};

  // TODO(naveenoid): Declare these as parameters in the context.
  const double g_;           // Gravitational acceleration (m/s^2).
  const double m_;           // Mass of the robot (kg).
  const double L_;           // Length of the arms (m).
  const double kF_;          // Force input constant.
  const double kM_;          // Moment input constant.
  const Eigen::Matrix3d I_;  // Moment of Inertia about the Center of Mass
};

/// Generates an LQR controller to move to @p nominal_position. Internally
/// computes the nominal input corresponding to a hover at position @p x0.
/// @see systems::LinearQuadraticRegulator.
std::unique_ptr<systems::AffineSystem<double>> StabilizingLQRController(
    const QuadrotorPlant<double>* quadrotor_plant,
    Eigen::Vector3d nominal_position);

}  // namespace quadrotor
}  // namespace examples

// The following code was added to prevent scalar conversion to symbolic scalar
// types. The QuadrotorPlant makes use of classes that are not compatible with
// the symbolic scalar. This NonSymbolicTraits is explained in
// drake/systems/framework/system_scalar_converter.h.
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<examples::quadrotor::QuadrotorPlant> : public NonSymbolicTraits {
};
}  // namespace scalar_conversion
}  // namespace systems

}  // namespace drake
