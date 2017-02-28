#include "drake/automotive/idm_planner.h"

#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

template <typename T>
T IdmPlanner<T>::Evaluate(
    const IdmPlannerParameters<T>& params,
    const T& ego_velocity,
    const T& target_distance,
    const T& target_distance_dot) {
  using std::pow;
  using std::sqrt;

  const T& v_ref = params.v_ref();
  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();
  DRAKE_DEMAND(a > 0.0);
  DRAKE_DEMAND(b > 0.0);
  DRAKE_DEMAND(target_distance > 0.0);
  const T& v_ego = ego_velocity;

  return a * (1.0 - pow(v_ego / v_ref, delta) -
              pow((s_0 + v_ego * time_headway +
                   v_ego * target_distance_dot / (2 * sqrt(a * b))) /
                      target_distance,
                  2.0));
}

template <typename T>
void IdmPlanner<T>::SetDefaultParameters(IdmPlannerParameters<T>* idm_params) {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  DRAKE_DEMAND(idm_params != nullptr);
  idm_params->set_v_ref(1.0);            // desired velocity in free traffic.
  idm_params->set_a(T(1.0));             // max acceleration.
  idm_params->set_b(T(3.0));             // comfortable braking deceleration.
  idm_params->set_s_0(T(1.0));           // minimum desired net distance.
  idm_params->set_time_headway(T(0.1));  // desired headway to lead vehicle.
  idm_params->set_delta(T(4.0));  // recommended choice of free-road exponent.
}

// These instantiations must match the API documentation in
// idm_planner.h.
template class IdmPlanner<double>;
template class IdmPlanner<drake::TaylorVarXd>;
template class IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
