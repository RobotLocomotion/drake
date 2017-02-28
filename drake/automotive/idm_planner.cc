#include "drake/automotive/idm_planner.h"

#include <cmath>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

template <typename T>
const T IdmPlanner<T>::Evaluate(const IdmPlannerParameters<T>& params,
                                const T& ego_velocity, const T& target_distance,
                                const T& target_distance_dot) {
  using std::pow;
  using std::sqrt;

  const T& v_ref = params.v_ref();
  const T& a = params.a();
  const T& b = params.b();
  const T& s_0 = params.s_0();
  const T& time_headway = params.time_headway();
  const T& delta = params.delta();

  DRAKE_DEMAND(a > 0.);
  DRAKE_DEMAND(b > 0.);
  DRAKE_DEMAND(target_distance > 0.);

  const T& v_ego = ego_velocity;

  // Compute the interaction acceleration terms.
  const T& closing_term = v_ego * target_distance_dot / (2 * sqrt(a * b));
  const T& small_distance_term = s_0 + v_ego * time_headway;

  const T& accel_interaction =
      pow((closing_term + small_distance_term) / target_distance, 2.);
  // Compute the free-road accleration term.
  const T& accel_free_road = pow(v_ego / v_ref, delta);

  // Compute the IDM acceleration.
  return a * (1. - accel_free_road - accel_interaction);
}

template <typename T>
void IdmPlanner<T>::SetDefaultParameters(IdmPlannerParameters<T>* idm_params) {
  // Default values from https://en.wikipedia.org/wiki/Intelligent_driver_model.
  DRAKE_DEMAND(idm_params != nullptr);
  idm_params->set_v_ref(10.);            // desired velocity in free traffic.
  idm_params->set_a(T(1.));              // max acceleration.
  idm_params->set_b(T(3.));              // comfortable braking deceleration.
  idm_params->set_s_0(T(1.));            // minimum desired net distance.
  idm_params->set_time_headway(T(0.1));  // desired headway to lead vehicle.
  idm_params->set_delta(T(4.));  // recommended choice of free-road exponent.
}

// These instantiations must match the API documentation in idm_planner.h.
template class IdmPlanner<double>;
// template class IdmPlanner<drake::TaylorVarXd>;
// template class IdmPlanner<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
