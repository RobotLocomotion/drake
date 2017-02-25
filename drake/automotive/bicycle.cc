#include "drake/automotive/bicycle.h"

#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

namespace {

// Specify the dimension of the state vector and of each input port.
static constexpr int kStateDimension{BicycleStateIndices::kNumCoordinates};
static constexpr int kSteeringInputDimension{1};
static constexpr int kForceInputDimension{1};

}  // namespace

template <typename T>
Bicycle<T>::Bicycle() {
  auto& steering_input =
      this->DeclareInputPort(systems::kVectorValued, kSteeringInputDimension);
  auto& force_input =
      this->DeclareInputPort(systems::kVectorValued, kForceInputDimension);
  auto& state_output =
      this->DeclareOutputPort(systems::kVectorValued, kStateDimension);
  this->DeclareContinuousState(1,                     // num_q (Ψ)
                               1,                     // num_v (Ψ_dot)
                               kStateDimension - 2);  // num_z (all but Ψ,
                                                      // Ψ_dot)

  steering_input_port_ = steering_input.get_index();
  force_input_port_ = force_input.get_index();
  state_output_port_ = state_output.get_index();
}

template <typename T>
Bicycle<T>::~Bicycle() {}

template <typename T>
const systems::InputPortDescriptor<T>& Bicycle<T>::get_steering_input_port()
    const {
  return systems::System<T>::get_input_port(steering_input_port_);
}

template <typename T>
const systems::InputPortDescriptor<T>& Bicycle<T>::get_force_input_port()
    const {
  return systems::System<T>::get_input_port(force_input_port_);
}

template <typename T>
const systems::OutputPortDescriptor<T>& Bicycle<T>::get_state_output_port()
    const {
  return systems::System<T>::get_output_port(state_output_port_);
}

template <typename T>
void Bicycle<T>::DoCalcOutput(const systems::Context<T>& context,
                              systems::SystemOutput<T>* output) const {
  // Obtain the state.
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const BicycleState<T>* const state =
      dynamic_cast<const BicycleState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  // Obtain the output pointer and mutate with our state.
  BicycleState<T>* const output_vector =
      dynamic_cast<BicycleState<T>*>(output->GetMutableVectorData(0));
  DRAKE_ASSERT(output_vector != nullptr);

  output_vector->set_value(state->get_value());
}

// Calculate the continuous-time derivatives.
template <typename T>
void Bicycle<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  // Obtain the parameters, states, inputs, and state derivatives.
  const int kParamsIndex = 0;
  const BicycleParameters<T>& params =
      this->template GetNumericParameter<BicycleParameters>(context,
                                                            kParamsIndex);
  const systems::VectorBase<T>& context_state =
      context.get_continuous_state_vector();
  const BicycleState<T>* const state =
      dynamic_cast<const BicycleState<T>*>(&context_state);
  DRAKE_ASSERT(state != nullptr);

  const systems::BasicVector<T>* steering =
      this->EvalVectorInput(context, get_steering_input_port().get_index());
  DRAKE_ASSERT(steering != nullptr);

  const systems::BasicVector<T>* force =
      this->EvalVectorInput(context, get_force_input_port().get_index());
  DRAKE_ASSERT(force != nullptr);

  DRAKE_ASSERT(derivatives != nullptr);
  systems::VectorBase<T>* derivative_vector = derivatives->get_mutable_vector();
  DRAKE_ASSERT(derivative_vector != nullptr);
  BicycleState<T>* const state_derivatives =
      dynamic_cast<BicycleState<T>*>(derivative_vector);
  DRAKE_ASSERT(state_derivatives != nullptr);

  ImplCalcTimeDerivatives(params, *state, *steering, *force, state_derivatives);
}

template <typename T>
void Bicycle<T>::ImplCalcTimeDerivatives(
    const BicycleParameters<T>& params,
    const BicycleState<T>& state,
    const systems::BasicVector<T>& steering,
    const systems::BasicVector<T>& force,
    BicycleState<T>* derivatives) const {
  // Parse and validate the parameters.
  const T m = params.mass();
  const T lr = params.lr();
  const T lf = params.lf();
  const T Iz = params.Iz();
  const T Cf = params.Cf();
  const T Cr = params.Cr();

  DRAKE_DEMAND(m > 0.);
  DRAKE_DEMAND(Iz > 0.);

  // Parse the inputs.
  const T delta = steering[0];
  const T F_in = force[0];

  // Parse and validate the states.
  const T Psi{state.Psi()};
  const T Psi_dot{state.Psi_dot()};
  const T beta{state.beta()};
  const T v{state.v()};

  DRAKE_DEMAND(v != 0.);  // N.B. Protection against the singular solution.
  // TODO(jadecastro): Enable v = 0. (see #5318).

  const T torsional_stiffness = Cr * lr - Cf * lf;
  const T front_torsional_stiffness = Cf * lf;
  const T torsional_damping = (Cf * pow(lf, 2.) + Cr * pow(lr, 2.)) / v;

  // Compute the differential equations of motion.
  const T Psi_ddot = torsional_stiffness / Iz * beta -
                     torsional_damping / Iz * Psi_dot +
                     front_torsional_stiffness / Iz * delta;
  const T beta_dot = (torsional_stiffness / (m * pow(v, 2.)) - 1.) * Psi_dot +
                     Cf / (m * v) * delta - (Cf + Cr) / (m * v) * beta;
  const T v_dot = F_in / m;
  const T sx_dot = v * cos(beta + Psi);
  const T sy_dot = v * sin(beta + Psi);

  derivatives->set_Psi(Psi_dot);
  derivatives->set_Psi_dot(Psi_ddot);
  derivatives->set_beta(beta_dot);
  derivatives->set_v(v_dot);
  derivatives->set_sx(sx_dot);
  derivatives->set_sy(sy_dot);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
Bicycle<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<BicycleState<T>>());
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> Bicycle<T>::AllocateOutputVector(
    const systems::OutputPortDescriptor<T>& descriptor) const {
  return std::make_unique<BicycleState<T>>();
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> Bicycle<T>::AllocateParameters() const {
  auto params = std::make_unique<BicycleParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void Bicycle<T>::SetDefaultParameters(const systems::LeafContext<T>& context,
                                      systems::Parameters<T>* params) const {
  // Parameters representative of a Cadillac SRX (from Althoff & Dolan, 2014).
  auto p = dynamic_cast<BicycleParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(p != nullptr);
  p->set_mass(T(2278.));  // Mass [kg].
  p->set_lf(T(1.292));    // Distance from center of mass to front axle [m].
  p->set_lr(T(1.515));    // Distance from center of mass to rear axle [m].
  p->set_Iz(T(3210.));    // Moment of inertia about the yaw-axis [kg m^2].
  p->set_Cf(T(10.8e4));   // Cornering stiffness (front) [N / rad].
  p->set_Cr(T(10.8e4));   // Cornering stiffness (rear) [N / rad].
}

// These instantiations must match the API documentation in bicycle.h.
template class Bicycle<double>;
template class Bicycle<AutoDiffXd>;
template class Bicycle<symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
