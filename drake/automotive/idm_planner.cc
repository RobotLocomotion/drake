#include "drake/automotive/idm_planner.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"

namespace drake {
namespace automotive {

template <typename T>
IdmPlanner<T>::IdmPlanner(const T& v_0) : v_0_(v_0) {
  this->DeclareInputPort(systems::kVectorValued,
                         2, //IdmPlannerInputIndices::kNumCoordinates,
                         systems::kContinuousSampling);
  this->DeclareInputPort(systems::kVectorValued,
                         2, //IdmPlannerInputIndices::kNumCoordinates,
                         systems::kContinuousSampling);
  this->DeclareOutputPort(systems::kVectorValued,
                          1,
                          systems::kContinuousSampling);
}

template <typename T>
IdmPlanner<T>::~IdmPlanner() {}

template <typename T>
void IdmPlanner<T>::EvalOutput(
    const systems::Context<T>& context,
    systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the structures we need to read from and write into.
  //const IdmPlannerInput<T>* const input_ego =
  //    dynamic_cast<const IdmPlannerInput<T>*>(
  //                                  this->EvalVectorInput(context, 0));
  //const IdmPlannerInput<T>* const input_agent =
  //    dynamic_cast<const IdmPlannerInput<T>*>(
  //                                  this->EvalVectorInput(context, 1));
  const auto input_ego = this->EvalVectorInput(context, 0);
  const auto input_agent = this->EvalVectorInput(context, 1);
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
      a * (1.0 - pow(input_ego->GetAtIndex(1) / v_0_, delta) -
           pow((s_0 + input_ego->GetAtIndex(1) * time_headway +
                input_ego->GetAtIndex(1) * (input_ego->GetAtIndex(1)
                   - input_agent->GetAtIndex(1)) /
                (2 * sqrt(a * b))) /
               (input_agent->GetAtIndex(0)
                   - input_ego->GetAtIndex(0) - l_a), 2.0)));
  /*
  output_vector->SetAtIndex(0,
      a * (1.0 - pow(input_ego->v() / v_0_, delta) -
           pow((s_0 + input_ego->v() * time_headway +
                input_ego->v() * (input_ego->v() - input_agent->v()) /
                (2 * sqrt(a * b))) /
               (input_agent->x() - input_ego->x() - l_a), 2.0)));
  */
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>>
IdmPlanner<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  return std::make_unique<systems::BasicVector<T>>(1 /* output vector size */);
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class DRAKE_EXPORT IdmPlanner<double>;
template class DRAKE_EXPORT IdmPlanner<drake::TaylorVarXd>;
  //template class DRAKE_EXPORT IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
