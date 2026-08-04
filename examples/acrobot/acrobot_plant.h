#pragma once

#include <memory>

#include "drake/examples/acrobot/acrobot_input.h"
#include "drake/examples/acrobot/acrobot_params.h"
#include "drake/examples/acrobot/acrobot_state.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {
namespace examples {
namespace acrobot {

/// @defgroup acrobot_systems Acrobot
/// @{
/// @brief Systems related to the Acrobot example.
/// @ingroup example_systems
/// @}

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
///
/// @system
/// name: AcrobotPlant
/// input_ports:
/// - elbow_torque (optional)
/// output_ports:
/// - acrobot_state
/// @endsystem
///
/// Note: If the elbow_torque input port is not connected, then the torque is
/// taken to be zero.
///
/// @tparam_default_scalar
/// @ingroup acrobot_systems
template <typename T>
class AcrobotPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotPlant);

  /// Constructs the plant.  The parameters of the system are stored as
  /// Parameters in the Context (see acrobot_params.h).
  AcrobotPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit AcrobotPlant(const AcrobotPlant<U>&);

  /// Sets the parameters to describe MIT Robot Locomotion Group's hardware
  /// acrobot.
  void SetMitAcrobotParameters(AcrobotParams<T>* parameters) const;

  ///@{
  /// Manipulator equation of Acrobot: M(q)q̈ + bias(q,q̇) = B*u.
  ///
  /// - M[2x2] is the mass matrix.
  /// - bias[2x1] includes the Coriolis term, gravity term and the damping term,
  ///   i.e. bias[2x1] = C(q,v)*v - τ_g(q) + [b1*q̇₁;b2*q̇₂].
  // TODO(russt): Update this to the newest conventions.
  Vector2<T> DynamicsBiasTerm(const systems::Context<T>& context) const;
  Matrix2<T> MassMatrix(const systems::Context<T>& context) const;
  ///@}

  /// Evaluates the input port and returns the scalar value of the commanded
  /// torque.  If the input port is not connected, then the torque is taken to
  /// be zero.
  const T get_tau(const systems::Context<T>& context) const {
    const systems::BasicVector<T>* u_vec = this->EvalVectorInput(context, 0);
    return u_vec ? u_vec->GetAtIndex(0) : 0.0;
  }

  static const AcrobotState<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const AcrobotState<T>&>(cstate.get_vector());
  }

  static const AcrobotState<T>& get_state(const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  static AcrobotState<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<AcrobotState<T>&>(cstate->get_mutable_vector());
  }

  static AcrobotState<T>& get_mutable_state(systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  const AcrobotParams<T>& get_parameters(
      const systems::Context<T>& context) const {
    return this->template GetNumericParameter<AcrobotParams>(context, 0);
  }

  AcrobotParams<T>& get_mutable_parameters(systems::Context<T>* context) const {
    return this->template GetMutableNumericParameter<AcrobotParams>(context, 0);
  }

 private:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;

  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void DoCalcImplicitTimeDerivativesResidual(
      const systems::Context<T>& context,
      const systems::ContinuousState<T>& proposed_derivatives,
      EigenPtr<VectorX<T>> residual) const override;
};

/// Constructs the Acrobot with (only) encoder outputs.
///
/// @system
/// name: AcrobotWEncoder
/// input_ports:
/// - elbow_torque
/// output_ports:
/// - measured_joint_positions
/// - <span style="color:gray">acrobot_state</span>
/// @endsystem
///
/// The `acrobot_state` output port is present only if the construction
/// parameter `acrobot_state_as_second_output` is true.
///
/// @ingroup acrobot_systems
template <typename T>
class AcrobotWEncoder : public systems::Diagram<T> {
 public:
  explicit AcrobotWEncoder(bool acrobot_state_as_second_output = false);

  const AcrobotPlant<T>* acrobot_plant() const { return acrobot_plant_; }

  AcrobotState<T>& get_mutable_acrobot_state(
      systems::Context<T>* context) const;

 private:
  AcrobotPlant<T>* acrobot_plant_{nullptr};
};

/// Constructs the LQR controller for stabilizing the upright fixed point using
/// default LQR cost matrices which have been tested for this system.
/// @ingroup acrobot_systems
std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>& acrobot);

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
