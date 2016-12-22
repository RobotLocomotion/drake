#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include <algorithm>
#include <limits>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace bouncing_ball {

template <typename T>
BouncingBall<T>::BouncingBall() {
}

template <typename T>
T BouncingBall<T>::EvalGuard(const systems::Context<T>& context) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Evaluate the guard function.
  const systems::VectorBase<T>& state =
    context.get_continuous_state_vector();

  // The guard is satisfied (returns a non-positive value) when
  // the ball's position is less than or equal to zero and its
  // velocity is non-positive.
  return std::max(state.GetAtIndex(0), state.GetAtIndex(1));
}

template <typename T>
void BouncingBall<T>::PerformReset(systems::Context<T>* context) const {
  DRAKE_ASSERT(context != nullptr);
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(*context));

  // Define a pointer to the continuous state in the context.
  const auto result =
    context->get_mutable_continuous_state_vector();

  // Perform the reset: map the position to itself and negate the
  // velocity and attenuate by the coefficient of restitution.
  auto state = context->get_mutable_continuous_state_vector();
  result->SetAtIndex(1,
     -1.0 * this->restitution_coef_ * state->GetAtIndex(1));
}

template <typename T>
void BouncingBall<T>::DoCalcNextUpdateTime(const systems::Context<T>& context,
                                           systems::UpdateActions<T>* actions)
                                             const {
  using std::sqrt;

  // Get the state of the guard function.
  const systems::VectorBase<T>& state =
      context.get_continuous_state_vector();

  // Two cases: (1) the ball is in ballistic flight and (2) the ball is on
  // the ground and about to return upward.
  T x0 = state.GetAtIndex(0);
  const T v0 = state.GetAtIndex(1);
  if (x0 <= std::numeric_limits<double>::epsilon()) {
    // Case (2) encountered. Verify that ball is returning upward.
    DRAKE_DEMAND(v0 > 0.0);

    // Update x0 such that the ball is slightly above the ground.
    x0 = std::numeric_limits<double>::epsilon();
  }

  // The time that the ball will impact the ground is:
  // gt^2/2 + v0*t + x0 = 0
  // Solve the quadratic equation for t. We expect that b^2 >> 4ac in some
  // cases, which means that we must use a special algorithm to combat
  // cancellation error.
  const T a = Ball<T>::get_gravitational_acceleration()/2;
  const T b = v0;
  const T c = x0;
  const T disc = (b*b - 4*a*c);
  DRAKE_DEMAND(disc > 0.0);
  T r1 = (-b - sgn(b)*sqrt(disc))/(2*a);
  T r2 = c/(a*r1);

  // We want the smallest positive root.
  if (r1 <= 0.0)
    r1 = std::numeric_limits<T>::infinity();
  if (r2 <= 0.0)
    r2 = std::numeric_limits<T>::infinity();

  // Verify that the impact time is reasonable.
  DRAKE_DEMAND(std::min(r1, r2) > 0.0);

  // Compute the impact time.
  actions->time = context.get_time() + std::min(r1, r2);
  actions->events.push_back(systems::DiscreteEvent<T>());
  actions->events.back().action = systems::DiscreteEvent<T>::
                                                  kUnrestrictedUpdateAction;
}

template <typename T>
void BouncingBall<T>::DoCalcUnrestrictedUpdate(const systems::Context<T>&
                                               context, systems::State<T>*
                                               next_state) const {
  systems::VectorBase<T>* next_cstate = next_state->
                           get_mutable_continuous_state()->get_mutable_vector();

  // Get present state.
  const systems::VectorBase<T>& cstate = context.get_continuous_state()->
      get_vector();

  // Copy the present state to the new one.
  next_state->CopyFrom(context.get_state());

  // Verify that velocity is non-positive.
  DRAKE_DEMAND(cstate.GetAtIndex(1) <= 0.0);

  // Update the velocity.
  next_cstate->SetAtIndex(1, cstate.GetAtIndex(1) *
      this->restitution_coef_ * -1.);
}

}  // namespace bouncing_ball
}  // namespace drake
