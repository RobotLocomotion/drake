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

  const int kEgoCarOutputVectorSize = 2;
  const int kAgentCarOutputVectorSize = 2;
  const int kLinearAccelerationSize = 1;
  // Declare the ego car input port.
  this->DeclareInputPort(systems::kVectorValued, kEgoCarOutputVectorSize);
  // Declare the agent car input port.
  this->DeclareInputPort(systems::kVectorValued, kAgentCarOutputVectorSize);
  // Declare the output port.
  this->DeclareOutputPort(systems::kVectorValued, kLinearAccelerationSize);
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
void IdmPlanner<T>::DoCalcOutput(const systems::Context<T>& context,
                                 systems::SystemOutput<T>* output) const {
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
  auto params = std::make_unique<IdmPlannerParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void IdmPlanner<T>::SetDefaultParameters(const systems::LeafContext<T>& context,
                                         systems::Parameters<T>* params) const {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  auto idm_params = dynamic_cast<IdmPlannerParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(idm_params != nullptr);
  idm_params->set_v_ref(v_ref_);         // desired velocity in free traffic.
  idm_params->set_a(T(1.0));             // max acceleration.
  idm_params->set_b(T(3.0));             // comfortable braking deceleration.
  idm_params->set_s_0(T(1.0));           // minimum desired net distance.
  idm_params->set_time_headway(T(0.1));  // desired headway to lead vehicle.
  idm_params->set_delta(T(4.0));  // recommended choice of free-road exponent.
  idm_params->set_l_a(T(4.5));    // length of leading car.
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmPlanner<double>;
template class IdmPlanner<drake::TaylorVarXd>;
template class IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
