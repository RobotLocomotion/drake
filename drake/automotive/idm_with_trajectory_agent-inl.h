#pragma once

/// @file
/// Template method implementations for idm_with_trajectory_agent.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "idm_with_trajectory_agent.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/common/drake_assert.h"

namespace drake {
namespace cars {

template <typename T>
IdmWithTrajectoryAgent<T>::IdmWithTrajectoryAgent() {
  this->DeclareOutputPort(systems::kVectorValued,
                          IdmWithTrajectoryAgentStateIndices::kNumCoordinates,
                          systems::kContinuousSampling);
}

template <typename T>
IdmWithTrajectoryAgent<T>::~IdmWithTrajectoryAgent() {}

template <typename T>
void IdmWithTrajectoryAgent<T>::EvalOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the structure we need to write into.
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // TODO(david-german-tri): Remove this copy by allowing output ports to be
  // mere pointers to state variables (or cache lines).
  output_vector->get_mutable_value() =
      context.get_state().continuous_state->get_state().CopyToVector();
}

template <typename T>
void IdmWithTrajectoryAgent<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_state().continuous_state->get_state();
  const IdmWithTrajectoryAgentState<T>* const state =
      dynamic_cast<const IdmWithTrajectoryAgentState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const derivatives_state =
      derivatives->get_mutable_state();
  DRAKE_ASSERT(derivatives_state != nullptr);
  IdmWithTrajectoryAgentState<T>* const new_derivatives =
      dynamic_cast<IdmWithTrajectoryAgentState<T>*>(derivatives_state);
  DRAKE_ASSERT(new_derivatives != nullptr);

  // Taken from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  const double a{1.0};  // max acceleration.
  const double b{3.0};  // comfortable braking deceleration.
  const double v_0{50.0};  // desired velocity in free traffic.
  const double s_0{1.0};  // minimum desired net distance.
  const double time_headway{0.1};  // desired time headway to vehicle in front.
  const double delta{4.0};  // recommended choice of free-road exponent.
  const double l_a{4.5};  // length of leading car.

  new_derivatives->set_x_e(state->v_e());
  new_derivatives->set_v_e(
      a * (1.0 - pow(state->v_e() / v_0, delta) -
           pow((s_0 + state->v_e() * time_headway +
                state->v_e() * (state->v_e() - state->v_a()) /
                (2 * sqrt(a * b))) /
               (state->x_a() - state->x_e() - l_a), 2.0)));

  new_derivatives->set_x_a(state->v_a());
  new_derivatives->set_v_a(state->a_a());
  new_derivatives->set_a_a(T{0.0});
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
IdmWithTrajectoryAgent<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<IdmWithTrajectoryAgentState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
IdmWithTrajectoryAgent<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<IdmWithTrajectoryAgentState<T>>();
}

}  // namespace cars
}  // namespace drake
