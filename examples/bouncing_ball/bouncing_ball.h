#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/leaf_system.h"
#include "drake/examples/bouncing_ball/signed_distance_witness.h"

namespace drake {
namespace bouncing_ball {

/// Dynamical representation of the idealized hybrid dynamics
/// of a ball dropping from a height and bouncing on a surface.
///
/// Instantiated templates for the following scalar types @p T are provided:
/// - double
/// - AutoDiffXd
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
///
/// They are already available to link against in the containing library.
///
/// Inputs: no inputs.
/// States: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
/// Outputs: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
template <typename T>
class BouncingBall : public systems::LeafSystem<T> {
 public:
  /// Constructor for the BouncingBall system.
  BouncingBall() {
    this->DeclareContinuousState(1, 1, 0);
    this->DeclareVectorOutputPort(systems::BasicVector<T>(2),
                                  &BouncingBall::CopyStateOut);

    // Create the witness function.
    signed_distance_witness_ =
        std::make_unique<SignedDistanceWitnessFunction<T>>(*this);
  }

  /// Gets the signed acceleration due to gravity. Since initial positions
  /// correspond to heights, acceleration should be negative.
  double get_gravitational_acceleration() const { return -9.81; }

    /// Getter for the coefficient of restitution for this model.
  double get_restitution_coef() const { return restitution_coef_; }

 private:
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const {
    output->get_mutable_value() =
        context.get_continuous_state().CopyToVector();
  }

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    // Obtain the state.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();

    // Obtain the structure we need to write into.
    DRAKE_ASSERT(derivatives != nullptr);
    systems::VectorBase<T>& new_derivatives = derivatives->get_mutable_vector();

    new_derivatives.SetAtIndex(0, state.GetAtIndex(1));
    new_derivatives.SetAtIndex(1, T(get_gravitational_acceleration()));
  }

  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    Vector2<T> x0;
    x0 << 10.0, 0.0;  // initial state values.
    state->get_mutable_continuous_state().SetFromVector(x0);
  }

  void DoCalcUnrestrictedUpdate(const systems::Context<T>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<T>*>& events,
      systems::State<T>* next_state) const override {
    systems::VectorBase<T>& next_cstate =
        next_state->get_mutable_continuous_state().get_mutable_vector();

    // Get present state.
    const systems::VectorBase<T>& cstate =
        context.get_continuous_state().get_vector();

    // Copy the present state to the new one.
    next_state->CopyFrom(context.get_state());

    // Verify that velocity is non-positive.
    DRAKE_DEMAND(cstate.GetAtIndex(1) <= 0.0);

    // Update the velocity.
    next_cstate.SetAtIndex(
        1, cstate.GetAtIndex(1) * this->restitution_coef_ * -1.);
  }

  void DoGetWitnessFunctions(
      const systems::Context<T>& context,
      std::vector<const systems::WitnessFunction<T>*>* witnesses)
      const override {
    witnesses->push_back(signed_distance_witness_.get());
  }

  const double restitution_coef_ = 1.0;  // Coefficient of restitution.

  std::unique_ptr<SignedDistanceWitnessFunction<T>> signed_distance_witness_;
};

}  // namespace bouncing_ball
}  // namespace drake
