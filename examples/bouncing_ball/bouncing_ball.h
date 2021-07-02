#pragma once

#include <memory>
#include <vector>

#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/witness_function.h"

namespace drake {
namespace examples {
namespace bouncing_ball {

/// Dynamical representation of the idealized hybrid dynamics
/// of a ball dropping from a height and bouncing on a surface.
///
/// Inputs: no inputs.
/// States: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
/// Outputs: vertical position (state index 0) and velocity (state index 1) in
/// units of m and m/s, respectively.
///
/// @tparam_default_scalar
template <typename T>
class BouncingBall final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(BouncingBall);

  BouncingBall() : systems::LeafSystem<T>(
      systems::SystemTypeTag<BouncingBall>{}) {
    // Two state variables: q and v.
    auto state_index = this->DeclareContinuousState(1, 1, 0);

    // The state of the system is output.
    this->DeclareStateOutputPort(systems::kUseDefaultName, state_index);

    // Declare the witness function.
    signed_distance_witness_ = this->MakeWitnessFunction(
        "Signed distance",
        systems::WitnessFunctionDirection::kPositiveThenNonPositive,
        &BouncingBall::CalcSignedDistance,
        systems::UnrestrictedUpdateEvent<T>());
  }

  /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
  template <typename U>
  explicit BouncingBall(const BouncingBall<U>&) : BouncingBall<T>() {}

  /// Gets the signed acceleration due to gravity. Since initial positions
  /// correspond to heights, acceleration should be negative.
  double get_gravitational_acceleration() const { return -9.81; }

  /// Getter for the coefficient of restitution for this model.
  double get_restitution_coef() const { return restitution_coef_; }

 private:
  // A witness function to determine when the bouncing ball crosses the
  // boundary q = 0 from q > 0. Note that the witness function only triggers
  // when the signed distance is positive at the left hand side of an interval
  // (via WitnessFunctionDirection::kPositiveThenNonPositive). An "unrestricted
  // update" event is necessary to change the velocity of the system
  // discontinuously.
  T CalcSignedDistance(const systems::Context<T>& context) const {
    const systems::VectorBase<T>& xc = context.get_continuous_state_vector();
    return xc.GetAtIndex(0);
  }

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override {
    // Obtain the state.
    const systems::VectorBase<T>& state = context.get_continuous_state_vector();

    // Obtain the structure we need to write into.
    DRAKE_ASSERT(derivatives != nullptr);
    systems::VectorBase<T>& derivatives_vec = derivatives->get_mutable_vector();

    // Time derivative of position (state index 0) is velocity.
    derivatives_vec.SetAtIndex(0, state.GetAtIndex(1));

    // Time derivative of velocity (state index 1) is acceleration.
    derivatives_vec.SetAtIndex(1, T(get_gravitational_acceleration()));
  }

  void SetDefaultState(const systems::Context<T>&,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    Vector2<T> xc0;
    xc0 << 10.0, 0.0;  // initial state values.
    state->get_mutable_continuous_state().SetFromVector(xc0);
  }

  // Updates the velocity discontinuously to reverse direction. This method
  // is called by the Simulator when the signed distance witness function
  // triggers.
  void DoCalcUnrestrictedUpdate(const systems::Context<T>& context,
      const std::vector<const systems::UnrestrictedUpdateEvent<T>*>&,
      systems::State<T>* next_state) const override {
    systems::VectorBase<T>& next_cstate =
        next_state->get_mutable_continuous_state().get_mutable_vector();

    // Get present state.
    const systems::VectorBase<T>& cstate =
        context.get_continuous_state().get_vector();

    // Copy the present state to the new one.
    next_state->SetFrom(context.get_state());

    // Verify that velocity is non-positive.
    DRAKE_DEMAND(cstate.GetAtIndex(1) <= 0.0);

    // Update the velocity using Newtonian restitution (note that Newtonian
    // restitution can lead to unphysical energy gains, as described in
    // [Stronge 1991]). For this reason, other impact models are generally
    // preferable.
    //
    // [Stronge 1991]  W. J. Stronge. Unraveling paradoxical theories for rigid
    //                 body collisions. J. Appl. Mech., 58:1049-1055, 1991.
    next_cstate.SetAtIndex(
        1, cstate.GetAtIndex(1) * restitution_coef_ * -1.);
  }

  // The signed distance witness function is always active and, hence, always
  // returned.
  void DoGetWitnessFunctions(
      const systems::Context<T>&,
      std::vector<const systems::WitnessFunction<T>*>* witnesses)
      const override {
    witnesses->push_back(signed_distance_witness_.get());
  }

  const double restitution_coef_ = 1.0;  // Coefficient of restitution.

  // The witness function for computing the signed distance between the ball
  // and the ground.
  std::unique_ptr<systems::WitnessFunction<T>> signed_distance_witness_;
};

}  // namespace bouncing_ball
}  // namespace examples
}  // namespace drake
