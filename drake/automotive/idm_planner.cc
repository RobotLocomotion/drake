#include "drake/automotive/idm_planner.h"

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

#include "drake/automotive/gen/idm_planner_parameters.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

template <typename T>
IdmPlanner<T>::IdmPlanner(const T& v_ref) : v_ref_(v_ref) {
  // TODO(jadecastro): Remove v_ref from the constructor.
  // The reference velocity must be strictly positive.
  DRAKE_ASSERT(v_ref > 0);

  // Declare the ego car input port.
  this->DeclareInputPort(systems::kVectorValued,
                         2,  // Size of the ego car output vector.
                         systems::kContinuousSampling);
  // Declare the agent car input port.
  this->DeclareInputPort(systems::kVectorValued,
                         2,  // Size of the agent car output vector.
                         systems::kContinuousSampling);
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued,
                          1,  // We have a single linear acceleration value.
                          systems::kContinuousSampling);
}

template <typename T>
IdmPlanner<T>::~IdmPlanner() {}

template <typename T>
const systems::SystemPortDescriptor<T>& IdmPlanner<T>::get_ego_port() const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::SystemPortDescriptor<T>& IdmPlanner<T>::get_agent_port() const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
void IdmPlanner<T>::EvalOutput(const systems::Context<T>& context,
                               systems::SystemOutput<T>* output) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidOutput(output));

  // Obtain the input/output structures we need to read from and write into.
  const systems::BasicVector<T>* input_ego =
      this->EvalVectorInput(context, this->get_ego_port().get_index());
  const systems::BasicVector<T>* input_agent =
      this->EvalVectorInput(context, this->get_agent_port().get_index());
  systems::BasicVector<T>* const output_vector =
      output->GetMutableVectorData(0);
  DRAKE_ASSERT(output_vector != nullptr);

  // Obtain the parameters.
  const int kParamsIndex = 0;
  const IdmPlannerParameters<T>& params =
      this->template GetNumericParameter<IdmPlannerParameters>(context,
                                                               kParamsIndex);

  const T& v_ref = params.v_ref();
  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();
  const T& l_a = params.l_a();

  const T& x_ego = input_ego->GetAtIndex(0);
  const T& v_ego = input_ego->GetAtIndex(1);
  const T& x_agent = input_agent->GetAtIndex(0);
  const T& v_agent = input_agent->GetAtIndex(1);

  // Ensure that we are supplying the planner with sane parameters and
  // input values.
  DRAKE_DEMAND(a > 0.0);
  DRAKE_DEMAND(b > 0.0);
  DRAKE_DEMAND(x_agent > (l_a + x_ego));

  output_vector->SetAtIndex(
      0, a * (1.0 - pow(v_ego / v_ref, delta) -
              pow((s_0 + v_ego * time_headway +
                   v_ego * (v_ego - v_agent) / (2 * sqrt(a * b))) /
                      (x_agent - x_ego - l_a),
                  2.0)));
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> IdmPlanner<T>::AllocateParameters()
    const {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  params->set_v_ref(v_ref_);         // desired velocity in free traffic.
  params->set_a(T(1.0));             // max acceleration.
  params->set_b(T(3.0));             // comfortable braking deceleration.
  params->set_s_0(T(1.0));           // minimum desired net distance.
  params->set_time_headway(T(0.1));  // desired time headway to lead vehicle.
  params->set_delta(T(4.0));  // recommended choice of free-road exponent.
  params->set_l_a(T(4.5));    // length of leading car.
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmPlanner<double>;
template class IdmPlanner<drake::TaylorVarXd>;
template class IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
