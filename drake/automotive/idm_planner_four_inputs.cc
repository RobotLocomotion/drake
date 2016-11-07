#include "drake/automotive/idm_planner_four_inputs.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace automotive {

template <typename T>
IdmPlannerFourInputs<T>::IdmPlannerFourInputs(const T& v_0) : v_0_(v_0) {
  for (int i = 0; i < 4; i++) {
    this->DeclareInputPort(systems::kVectorValued,
                           1,
                           systems::kContinuousSampling);
  }
  this->DeclareOutputPort(systems::kVectorValued,
                          1,
                          systems::kContinuousSampling);
}

template <typename T>
IdmPlannerFourInputs<T>::~IdmPlannerFourInputs() {}

template <typename T>
void IdmPlannerFourInputs<T>::EvalOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the structures we need to read from and write into.
  const systems::BasicVector<T>* input_ego_pos
    = this->EvalVectorInput(context, 0);
  const systems::BasicVector<T>* input_ego_vel
    = this->EvalVectorInput(context, 1);
  const systems::BasicVector<T>* input_agent_pos
    = this->EvalVectorInput(context, 2);
  const systems::BasicVector<T>* input_agent_vel
    = this->EvalVectorInput(context, 3);
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // TODO: bake in David's parameters definition stuff.
  IdmPlannerParameters<T> params;
  params.set_a(T(1.0));  // max acceleration.
  params.set_b(T(3.0));  // comfortable braking deceleration.
  params.set_s_0(T(1.0));  // minimum desired net distance.
  params.set_time_headway(T(0.1));  // desired time headway to vehicle in front.
  params.set_delta(T(4.0));  // recommended choice of free-road exponent.
  params.set_l_a(T(4.5));  // length of leading car.

  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();
  const T& l_a = params.l_a();

  output_vector->SetAtIndex(0,
     a * (1.0 - pow(input_ego_vel->GetAtIndex(0) / v_0_, delta) -
          pow((s_0 + input_ego_vel->GetAtIndex(0) * time_headway +
               input_ego_vel->GetAtIndex(0) *
               (input_ego_vel->GetAtIndex(0) - input_agent_vel->GetAtIndex(0)) /
                (2 * sqrt(a * b))) /
              (input_agent_pos->GetAtIndex(0)
               - input_ego_pos->GetAtIndex(0) - l_a), 2.0)));
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
IdmPlannerFourInputs<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<systems::BasicVector<T>>(1 /* output vector size */);
}

// These instantiations must match the API documentation in
// idm_planner_four_inputs.h.
template class DRAKE_EXPORT IdmPlannerFourInputs<double>;
template class DRAKE_EXPORT IdmPlannerFourInputs<drake::TaylorVarXd>;
  //template class DRAKE_EXPORT
  //  IdmPlannerFourInputs<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
