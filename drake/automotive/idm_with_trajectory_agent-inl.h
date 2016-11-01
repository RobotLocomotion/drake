#pragma once

/// @file
/// Template method implementations for idm_with_trajectory_agent.h.
/// Most users should only include that file, not this one.
/// For background, see http://drake.mit.edu/cxx_inl.html.

#include "drake/automotive/idm_with_trajectory_agent.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/gen/idm_with_trajectory_agent_parameters.h"
#include "drake/common/drake_assert.h"

namespace drake {
namespace automotive {

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
      context.get_continuous_state()->CopyToVector();
}

template <typename T>
void IdmWithTrajectoryAgent<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const IdmWithTrajectoryAgentState<T>* const state =
      dynamic_cast<const IdmWithTrajectoryAgentState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtain the structure we need to write into.
  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* const derivatives_state =
      derivatives->get_mutable_vector();
  DRAKE_ASSERT(derivatives_state != nullptr);
  IdmWithTrajectoryAgentState<T>* const new_derivatives =
      dynamic_cast<IdmWithTrajectoryAgentState<T>*>(derivatives_state);
  DRAKE_ASSERT(new_derivatives != nullptr);

  // Taken from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  IdmWithTrajectoryAgentParameters<T> params;
  params.set_a(T(1.0));  // max acceleration.
  params.set_b(T(3.0));  // comfortable braking deceleration.
  params.set_v_0(T(50.0));  // desired velocity in free traffic.
  params.set_s_0(T(1.0));  // minimum desired net distance.
  params.set_time_headway(T(0.1));  // desired time headway to vehicle in front.
  params.set_delta(T(4.0));  // recommended choice of free-road exponent.
  params.set_l_a(T(4.5));  // length of leading car.

  const T& a = params.a();
  const T& b = params.b();
  const T& v_0 = params.v_0();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();
  const T& l_a = params.l_a();

  // NOLINTNEXTLINE(build/namespaces) For ADL of pow.
  using namespace std;

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

}  // namespace automotive
}  // namespace drake
