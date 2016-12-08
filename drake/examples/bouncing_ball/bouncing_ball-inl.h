#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/examples/bouncing_ball/bouncing_ball.h"

#include <algorithm>

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
                          systems::UpdateActions<T>* actions) const {
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
void BouncingBall<T>::DoPerformUnrestrictedUpdate(systems::Context<T>* context)
                                                            const {
  // Verify that velocity is non-positive.
  const systems::VectorBase<T>& state =
      context->get_continuous_state_vector();
  DRAKE_DEMAND(state.GetAtIndex(1) <= 0.0);

  // Call the reset function.
  PerformReset(context);
}

template <typename T>
T BouncingBall<T>::CalcClosedFormHeight(const T x0, const T tf) {
  using std::sqrt;

  // The time that the ball will impact the ground is:
  // gt^2/2 + x0 = 0
    // Solve the quadratic equation for t.
  const T a = Ball<T>::get_gravitational_acceleration()/2;
  const T c = x0;
  T drop_time = sqrt(-c/a);

  // Handle the cases appropriately.
  if (get_restitution_coef() == T(0)) {
    // TODO(edrumwri): Test these cases when we can handle the Zeno's Paradox
    // problem.
    if (tf < drop_time) {
      // In a ballistic phase.
      return Ball<T>::get_gravitational_acceleration()*tf*tf*0.5 + x0;
    } else {
      // Ball has hit the ground.
      return T(0);
    }
  } else {
    if (get_restitution_coef() == T(1)) {
      // Get the number of phases that have passed.
      using std::floor;
      int num_phases = static_cast<int>(drake::TtoDouble<T>::convert(
          floor(tf / drop_time)));

      // Get the time within the phase.
      const T t = tf - num_phases*drop_time;

      // Even phases mean that the ball is falling, odd phases mean that it is
      // rising.
      if ((num_phases & 1) == 0) {
        return Ball<T>::get_gravitational_acceleration()*t*t*0.5 + x0;
      } else {
        // Get the ball velocity at the time of impact.
        const T vf = Ball<T>::get_gravitational_acceleration()*drop_time;
        return Ball<T>::get_gravitational_acceleration()*t*t*0.5 - vf*t;
      }
    } else {
      throw std::logic_error("Invalid restitution coefficient!");
    }

    // Should never reach here.
    DRAKE_ABORT();
    return 0.0;
  }
}

template <typename T>
T BouncingBall<T>::CalcClosedFormVelocity(const T x0, const T tf) {
  using std::sqrt;

  // The time that the ball will impact the ground is:
  // gt^2/2 + x0 = 0
  // Solve the quadratic equation for t.
  const T a = Ball<T>::get_gravitational_acceleration()/2;
  const T c = x0;
  T drop_time = sqrt(-c/a);

  // Handle the cases appropriately.
  if (get_restitution_coef() == T(0)) {
    // TODO(edrumwri): Test these cases when we can handle the Zeno's Paradox
    // problem.
    if (tf < drop_time) {
      // In a ballistic phase.
      return Ball<T>::get_gravitational_acceleration()*tf;
    } else {
      // Ball has hit the ground.
      return T(0);
    }
  } else {
    if (get_restitution_coef() == T(1)) {
      // Get the number of phases that have passed.
      using std::floor;
      int num_phases = static_cast<int>(drake::TtoDouble<T>::convert(
          floor(tf / drop_time)));

      // Get the time within the phase.
      const T t = tf - num_phases*drop_time;

      // Even phases mean that the ball is falling, odd phases mean that it is
      // rising. Horrible bit of code below is due to AutoDiff. Thanks AutoDiff!
      if ((num_phases & 1) == 0) {
        return Ball<T>::get_gravitational_acceleration()*t;
      } else {
        // Get the ball velocity at the time of impact.
        const T vf = Ball<T>::get_gravitational_acceleration()*drop_time;
        return Ball<T>::get_gravitational_acceleration()*t - vf;
      }
    } else {
      throw std::logic_error("Invalid restitution coefficient!");
    }

    // Should never reach here.
    DRAKE_ABORT();
    return 0.0;
  }
}

}  // namespace bouncing_ball
}  // namespace drake
