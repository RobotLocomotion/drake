#include "drake/automotive/dynamic_bicycle_car.h"

namespace drake {
namespace automotive {

template <typename T>
DynamicBicycleCar<T>::DynamicBicycleCar()
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<automotive::DynamicBicycleCar>{}) {
  this->DeclareVectorInputPort(DynamicBicycleCarInput<T>());
  this->DeclareVectorOutputPort(DynamicBicycleCarState<T>(),
                                &DynamicBicycleCar::CopyStateOut);

  // Declares that this system has a continuous state of size
  // DynamicBicycleCarState.size() and in a vector cloned from
  // DynamicBicycleCarState.
  this->DeclareContinuousState(DynamicBicycleCarState<T>());

  // Declares the systems numeric parameters from the named vector
  // dynamic_bicycle_car_params.named_vector.
  this->DeclareNumericParameter(DynamicBicycleCarParams<T>());
}

template <typename T>
const systems::OutputPort<T>& DynamicBicycleCar<T>::get_output_port() const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>& DynamicBicycleCar<T>::get_input_port()
    const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const DynamicBicycleCarState<T>& DynamicBicycleCar<T>::get_state(
    const systems::Context<T>& context) {
  const systems::ContinuousState<T>& cstate = context.get_continuous_state();
  // Casts the continuous state vector from a VectorBase to a
  // DynamicBicycleState vector.
  return dynamic_cast<const DynamicBicycleCarState<T>&>(cstate.get_vector());
}

template <typename T>
DynamicBicycleCarState<T>& DynamicBicycleCar<T>::get_mutable_state(
    systems::Context<T>* context) {
  systems::ContinuousState<T>* cstate =
      &context->get_mutable_continuous_state();
  return dynamic_cast<DynamicBicycleCarState<T>&>(cstate->get_mutable_vector());
}

template <typename T>
void DynamicBicycleCar<T>::CopyStateOut(
    const systems::Context<T>& context,
    DynamicBicycleCarState<T>* output) const {
  output->set_value(get_state(context).get_value());
}

template <typename T>
const T DynamicBicycleCar<T>::CalcTireSlip(
    const DynamicBicycleCarState<T>& state,
    const DynamicBicycleCarParams<T>& params, const T& steer_angle,
    bool tire) const {
  using std::atan2;

  if (tire) {
    // Front tire slip angle.
    return atan2(state.v_LCp_y() + params.Lf() * state.yawDt_LC(),
                 state.v_LCp_x()) - steer_angle;
  } else {
    // Rear tire slip angle.
    return atan2(state.v_LCp_y() - params.Lb() * state.yawDt_LC(),
                 state.v_LCp_x());
  }
}

template <typename T>
const T DynamicBicycleCar<T>::CalcNormalTireForce(
    const DynamicBicycleCarParams<T>& params, const T f_Cp_x, bool tire) const {
  if (tire) {
    // Normal force for the front tires.
    return (1 / (params.Lf() + params.Lb())) *
           (params.mass() * params.Lb() * params.gravity() -
            params.h_cm() * f_Cp_x);
  } else {
    // Normal force for the rear tires.
    return (1 / (params.Lf() + params.Lb())) *
           (params.mass() * params.Lf() * params.gravity() +
            params.h_cm() * f_Cp_x);
  }
}

template <typename T>
const T DynamicBicycleCar<T>::CalcLateralTireForce(const T tire_slip_angle,
                                                   const T c_alpha, const T f_z,
                                                   const T mu) const {
  // Based on Fiala non-linear brush tire model as presented by Pacejka.
  using std::pow;
  using std::tan;
  using std::abs;
  using std::atan2;

  T f_y_non_saturated_tire = -c_alpha * tan(tire_slip_angle) +
                            ((c_alpha * c_alpha) / (3 * mu * f_z)) *
                                abs(tan(tire_slip_angle)) *
                                tan(tire_slip_angle) -
                            (pow(c_alpha, 3) / (27 * (mu * mu) * (f_z * f_z))) *
                                pow(tan(tire_slip_angle), 3);
  T f_y_saturated_tire = -mu * f_z * abs(tire_slip_angle) / tire_slip_angle;

  // Note: the cond function is used as an if-else statement in order to make
  // the conditional symbolic::Expression capable.
  return cond(abs(tire_slip_angle) < atan2(3 * mu * f_z, c_alpha),
              f_y_non_saturated_tire, f_y_saturated_tire);
}

template <typename T>
void DynamicBicycleCar<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Get the current state and derivative vectors of the system.
  const DynamicBicycleCarState<T>& state = get_state(context);
  DynamicBicycleCarState<T>& derivative_state =
      dynamic_cast<DynamicBicycleCarState<T>&>(
          derivatives->get_mutable_vector());

  // Obtain the car parameters.
  const DynamicBicycleCarParams<T>& params =
      this->template GetNumericParameter<DynamicBicycleCarParams>(context, 0);

  // Get the inputs.
  const T steer_CD = get_steer(context);
  const T f_Cp_x = get_longitudinal_force(context);

  const bool front_tire = true;
  const bool rear_tire = false;

  // Calculate tire slip angles.
  const T tire_slip_angle_f =
      CalcTireSlip(state, params, steer_CD, front_tire);
  const T tire_slip_angle_r =
      CalcTireSlip(state, params, steer_CD, rear_tire);

  // Calculate tire forces.
  const T f_z_f = CalcNormalTireForce(params, f_Cp_x, front_tire);
  const T f_z_r = CalcNormalTireForce(params, f_Cp_x, rear_tire);
  const T f_y_f = CalcLateralTireForce(tire_slip_angle_f, params.c_alpha_f(),
                                       f_z_f, params.mu());
  const T f_y_r = CalcLateralTireForce(tire_slip_angle_r, params.c_alpha_r(),
                                       f_z_r, params.mu());

  T sideslip = 0;
  // Catch to calculate sideslip angle when v_LoCp_x drops below 1 m/s.
  // Note: the cond function is used as an if-else statement in order to make
  // the conditional symbolic::Expression capable.
  sideslip = cond(state.v_LCp_x() < 1, state.v_LCp_y() / 1,
                  state.v_LCp_y() / state.v_LCp_x());

  // Calculate state derivatives.
  using std::sin;
  using std::cos;
  derivative_state.set_p_LoCp_x(state.v_LCp_x());
  derivative_state.set_p_LoCp_y(state.v_LCp_y());
  derivative_state.set_yaw_LC(state.yawDt_LC());
  derivative_state.set_v_LCp_x(
      (f_Cp_x / params.mass()) + state.yawDt_LC() * state.v_LCp_x() * sideslip);
  derivative_state.set_v_LCp_y((f_y_f * cos(steer_CD) + f_y_r) /
                                      params.mass() -
                                  state.yawDt_LC() * state.v_LCp_x());
  derivative_state.set_yawDt_LC(
      (params.Lf() * f_y_f * cos(steer_CD) - params.Lb() * f_y_r) /
      params.izz());
}

}  // namespace automotive
}  // namespace drake

// Explicitly instantiate on non-symbolic scalar types.
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::automotive::DynamicBicycleCar)
