#pragma once

/// @file
/// Template method implementations for bouncing_ball.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

/* clang-format off to disable clang-format-includes */
#include "drake/examples/bouncing_ball/bouncing_ball.h"
/* clang-format on */

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace bouncing_ball {

template <typename T>
BouncingBall<T>::BouncingBall() {
  signed_distance_witness_ =
      std::make_unique<SignedDistanceWitnessFunction<T>>(*this);
}

template <class T>
void BouncingBall<T>::DoGetWitnessFunctions(
    const systems::Context<T>&,
    std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
  witnesses->push_back(signed_distance_witness_.get());
}

template <typename T>
void BouncingBall<T>::DoCalcUnrestrictedUpdate(
    const systems::Context<T>& context,
    const std::vector<const systems::UnrestrictedUpdateEvent<T>*>&,
    systems::State<T>* next_state) const {
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
  next_cstate.SetAtIndex(1,
                         cstate.GetAtIndex(1) * this->restitution_coef_ * -1.);
}

}  // namespace bouncing_ball
}  // namespace drake
