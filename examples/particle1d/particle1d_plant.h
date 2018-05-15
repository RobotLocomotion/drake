#pragma once

#include <memory>

#include "drake/common/autodiff.h"
#include "drake/examples/particle1d/particle1d_manual.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace particle1d {

/// Simple one-dimensional (x-direction) motion of a 1 kg particle (rigid box)
/// that is harmonically forced by @f[f = cos(t)@f]. The ODE governing the
/// block's x-position is  @f[f = m * ẍ@f], which specializes to
/// @f[ẍ = cos(t)@f]. The analytical (closed-form) solution for the block's
/// position is: @f[x(t) = x(0) + (1-cos(t)) + ẋ(0) * t@f].
///
/// This class provides a simple example of creating a LeafSystem that directly
/// uses hand-generated equations (in place of
/// the kinematics, dynamics, and related algorithms in RigidBodyPlant).
/// This LeafSystem is used by the Diagram and Simulator classes.
/// This LeafSystem has one output port (for outputing the state at some time
/// t), but does not have any inputs port (time is passed in through the
/// Context).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
template <typename T>
class Particle1dPlant final : public systems::LeafSystem<T> {
 public:
  // Disables the built in copy and move constructor.
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Particle1dPlant);

  /// Constructor defines a prototype for its continuous state and output port.
  Particle1dPlant();

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit Particle1dPlant(const Particle1dPlant<U>&) : Particle1dPlant<T>() {}

  /// Return the OutputPort associated with this Particle1dPlant.
  /// This method is called when connecting the ports in the diagram builder.
  const systems::OutputPort<T>& get_output_port() const {
    return systems::System<T>::get_output_port(0);
  }

  /// Set constant parameters (such as body mass) after class construction
  /// (but before calculations).
  /// @param[in] tree A RigidBodyTree that contains a model of the particle
  /// from the urdf file.
  void SetConstantParameters(const RigidBodyTree<double>& tree);

  /// Returns the current state of the Particle1dPlant as a BasicVector.
  /// This method is called when building a diagram in order to get access to
  /// the state of the system as given by the System Context.
  /// @param[in] context The Particle1dPlant sub-system context as given by the
  /// simulator's system context.
  static systems::BasicVector<T>& get_mutable_state(
      systems::Context<T>* context) {
    return get_mutable_state(&context->get_mutable_continuous_state());
  }

  /// Returns the mass of the particle in kg.
  const T get_mass() const { return particle1d_.get_particle_data().mass; }

 private:
  // Casts the continuous state vector from a VectorBase to a BasicVector
  static const systems::BasicVector<T>& get_state(
      const systems::ContinuousState<T>& cstate) {
    return dynamic_cast<const systems::BasicVector<T>&>(cstate.get_vector());
  }

  // This method is called in DoCalcTimeDerivative as a way to update the state
  // before the time derivatives are calculated.
  static const systems::BasicVector<T>& get_state(
      const systems::Context<T>& context) {
    return get_state(context.get_continuous_state());
  }

  // Casts the continuous state vector from a VectorBase to a BasicVector
  static systems::BasicVector<T>& get_mutable_state(
      systems::ContinuousState<T>* cstate) {
    return dynamic_cast<systems::BasicVector<T>&>(cstate->get_mutable_vector());
  }

  // Class that holds particle parameters and equations of motion.
  mutable Particle1dManual<T> particle1d_;

  // This is the calculator method that assigns values to the state output port.
  void CopyStateOut(const systems::Context<T>& context,
                                        systems::BasicVector<T>* output) const;

  // Method that calculates the state time derivatives
  void DoCalcTimeDerivatives(const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;
};

} // namespace particle1d
} // namespace examples

// Explicitly disable symbolic::Expression.
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<examples::particle1d::Particle1dPlant>
  : public NonSymbolicTraits {};

}  // namespace scalar_conversion
}  // namespace systems

} // namespace drake