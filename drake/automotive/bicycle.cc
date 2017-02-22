#include "drake/automotive/bicycle.h"

#include <cmath>
#include <memory>
#include <utility>

#include <Eigen/Geometry>

#include "drake/automotive/gen/bicycle_parameters.h"
#include "drake/common/autodiff_overloads.h"
#include "drake/common/drake_assert.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/common/symbolic_formula.h"

namespace drake {
namespace automotive {

template <typename T>
Bicycle<T>::Bicycle() {
  this->DeclareInputPort(systems::kVectorValued, kSteeringInputDimension);
  this->DeclareInputPort(systems::kVectorValued, kForceInputDimension);
  this->DeclareOutputPort(systems::kVectorValued, kStateDimension);
  this->DeclareContinuousState(1,   // num_q (Psi)
                               1,   // num_v (Psi_dot)
                               kStateDimension - 2);  // num_z
}

template <typename T>
Bicycle<T>::~Bicycle() {}

template <typename T>
const systems::InputPortDescriptor<T>& Bicycle<T>::get_steering_input_port()
    const {
  return systems::System<T>::get_input_port(0);
}

template <typename T>
const systems::InputPortDescriptor<T>& Bicycle<T>::get_force_input_port()
    const {
  return systems::System<T>::get_input_port(1);
}

template <typename T>
const systems::OutputPortDescriptor<T>& Bicycle<T>::get_state_output_port()
    const {
  return systems::System<T>::get_output_port(0);
}

template <typename T>
void Bicycle<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  Vector6<T> state = this->CopyContinuousStateVector(context);
  this->GetMutableOutputVector(output, get_state_output_port().get_index()) =
      state;
}

// Calculate the continuous-time derivatives.
template <typename T>
void Bicycle<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));
  DRAKE_ASSERT(derivatives != nullptr);

  // Obtain the state and input vectors.
  const Vector6<T> state = context.get_continuous_state_vector().CopyToVector();

  const systems::BasicVector<T>* steering_ptr =
      this->EvalVectorInput(context, get_steering_input_port().get_index());
  DRAKE_ASSERT(steering_ptr != nullptr);
  const T delta = steering_ptr->GetAtIndex(0);

  const systems::BasicVector<T>* force_ptr =
      this->EvalVectorInput(context, get_force_input_port().get_index());
  DRAKE_ASSERT(force_ptr != nullptr);
  const T F_in = force_ptr->GetAtIndex(0);

  // Obtain the parameters.
  const int kParamsIndex = 0;
  const BicycleParameters<T>& params =
      this->template GetNumericParameter<BicycleParameters>(context,
                                                            kParamsIndex);
  const T m = params.mass();
  const T lr = params.lf();
  const T lf = params.lr();
  const T Iz = params.Iz();
  const T Cf = params.Cf();
  const T Cr = params.Cr();

  DRAKE_DEMAND(m > 0.);
  DRAKE_DEMAND(Iz > 0.);

  // Obtain the states.
  const T Psi{state(0)};
  const T Psi_dot{state(1)};
  const T beta{state(2)};
  const T v{state(3)};

  DRAKE_DEMAND(v != 0.);

  const T torsional_stiffness = Cr * lr - Cf * lf;
  const T front_torsional_stiffness = Cf * lf;
  const T torsional_damping = (Cf * pow(lf, 2.) + Cr * pow(lr, 2.)) / v;

  // Compute the differential equations of motion.
  const T Psi_ddot = torsional_stiffness / Iz * beta -
      torsional_damping / Iz * Psi_dot + front_torsional_stiffness / Iz * delta;
  const T beta_dot = (torsional_stiffness / (m * pow(v, 2.)) - 1.) * Psi_dot +
      Cf / (m * v) * delta  - (Cf + Cr) / (m * v) * beta;
  const T v_dot = F_in / m;
  const T sx_dot = v * cos(beta + Psi);
  const T sy_dot = v * sin(beta + Psi);

  Vector6<T> state_dot;
  state_dot << Psi_dot, Psi_ddot, beta_dot, v_dot, sx_dot, sy_dot;
  derivatives->SetFromVector(state_dot);
}

template <typename T>
std::unique_ptr<systems::Parameters<T>> Bicycle<T>::AllocateParameters() const {
  auto params = std::make_unique<BicycleParameters<T>>();
  return std::make_unique<systems::Parameters<T>>(std::move(params));
}

template <typename T>
void Bicycle<T>::SetDefaultParameters(const systems::LeafContext<T>& context,
                                      systems::Parameters<T>* params) const {
  // Parameters taken from Althoff & Dolan, 2014
  auto p = dynamic_cast<BicycleParameters<T>*>(
      params->get_mutable_numeric_parameter(0));
  DRAKE_DEMAND(p != nullptr);
  p->set_mass(T(2278.));  // Mass [kg].
  p->set_lf(T(1.292));  // Distance from CG to front axle [m].
  p->set_lr(T(1.515));  // Distance from CG to rear axle [m].
  p->set_Iz(T(3210.));  // Moment of inertia about the yaw-axis [kg m^2].
  p->set_Cf(T(10.8e4));  // Cornering stiffness (front) [N / rad].
  p->set_Cr(T(10.8e4));  // Cornering stiffness (rear) [N / rad].
}

// These instantiations must match the API documentation in bicycle.h.
template class Bicycle<double>;
template class Bicycle<drake::AutoDiffXd>;
template class Bicycle<drake::symbolic::Expression>;

}  // namespace automotive
}  // namespace drake
