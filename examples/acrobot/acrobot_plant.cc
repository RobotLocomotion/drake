#include "drake/examples/acrobot/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rotary_encoders.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace acrobot {

template <typename T>
AcrobotPlant<T>::AcrobotPlant()
    : systems::LeafSystem<T>(systems::SystemTypeTag<AcrobotPlant>{}) {
  this->DeclareNumericParameter(AcrobotParams<T>());
  this->DeclareVectorInputPort("elbow_torque", AcrobotInput<T>());
  auto state_index = this->DeclareContinuousState(
      AcrobotState<T>(), 2 /* num_q */, 2 /* num_v */, 0 /* num_z */);
  this->DeclareStateOutputPort("acrobot_state", state_index);
}

template <typename T>
template <typename U>
AcrobotPlant<T>::AcrobotPlant(const AcrobotPlant<U>&) : AcrobotPlant<T>() {}

template <typename T>
void AcrobotPlant<T>::SetMITAcrobotParameters(systems::Parameters<T>*
parameters) const {
  auto* p = dynamic_cast<AcrobotParams<T>*>(parameters);
  DRAKE_ASSERT(p != nullptr);
  p->set_m1(2.4367);
  p->set_m2(0.6178);
  p->set_l1(0.2563);
  p->set_lc1(1.6738);
  p->set_lc2(1.5651);
  p->set_Ic1(-4.7443);  // Notes: Yikes!  Negative inertias (from sysid).
  p->set_Ic2(-1.0068);
  p->set_b1(0.0320);
  p->set_b2(0.0413);
  // Note: parameters are identified in a way that torque has the unit of
  // current (Amps), in order to simplify the implementation of torque
  // constraint on motors. Therefore, some of the numbers here have incorrect
  // units.
}

template <typename T>
Matrix2<T> AcrobotPlant<T>::MassMatrix(
    const systems::Context<T> &context) const {
  const AcrobotState<T>& state = get_state(context);
  const AcrobotParams<T>& p = get_parameters(context);
  const T c2 = cos(state.theta2());
  const T I1 = p.Ic1() + p.m1() * p.lc1() * p.lc1();
  const T I2 = p.Ic2() + p.m2() * p.lc2() * p.lc2();
  const T m2l1lc2 = p.m2() * p.l1() * p.lc2();

  const T m12 = I2 + m2l1lc2 * c2;
  Matrix2<T> M;
  M << I1 + I2 + p.m2() * p.l1() * p.l1() + 2 * m2l1lc2 * c2, m12, m12,
      I2;
  return M;
}

template <typename T>
Vector2<T> AcrobotPlant<T>::DynamicsBiasTerm(const systems::Context<T> &
context)
const {
  const AcrobotState<T>& state = get_state(context);
  const AcrobotParams<T>& p = get_parameters(context);

  const T s1 = sin(state.theta1()), s2 = sin(state.theta2());
  const T s12 = sin(state.theta1() + state.theta2());
  const T m2l1lc2 = p.m2() * p.l1() * p.lc2();

  Vector2<T> bias;
  // C(q,v)*v terms.
  bias << -2 * m2l1lc2 * s2 * state.theta2dot() * state.theta1dot() +
           -m2l1lc2 * s2 * state.theta2dot() * state.theta2dot(),
      m2l1lc2 * s2 * state.theta1dot() * state.theta1dot();

  // -τ_g(q) terms.
  bias(0) += p.gravity() * p.m1() * p.lc1() * s1 +
          p.gravity() * p.m2() * (p.l1() * s1 + p.lc2() * s12);
  bias(1) += p.gravity() * p.m2() * p.lc2() * s12;

  // Damping terms.
  bias(0) += p.b1() * state.theta1dot();
  bias(1) += p.b2() * state.theta2dot();

  return bias;
}

// Compute the actual physics.
template <typename T>
void AcrobotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const AcrobotState<T>& state = get_state(context);
  const T& tau = get_tau(context);

  const Matrix2<T> M = MassMatrix(context);
  const Vector2<T> bias = DynamicsBiasTerm(context);
  const Vector2<T> B(0, 1);  // input matrix

  Vector4<T> xdot;
  xdot << state.theta1dot(), state.theta2dot(),
          M.inverse() * (B * tau - bias);
  derivatives->SetFromVector(xdot);
}

template <typename T>
void AcrobotPlant<T>::DoCalcImplicitTimeDerivativesResidual(
    const systems::Context<T>& context,
    const systems::ContinuousState<T>& proposed_derivatives,
    EigenPtr<VectorX<T>> residual) const {
  DRAKE_DEMAND(residual != nullptr);
  const AcrobotState<T>& state = get_state(context);
  const T& tau = get_tau(context);

  const Matrix2<T> M = MassMatrix(context);
  const Vector2<T> bias = DynamicsBiasTerm(context);
  const Vector2<T> B(0, 1);  // input matrix

  const auto& proposed_qdot = proposed_derivatives.get_generalized_position();
  const auto proposed_vdot =
      proposed_derivatives.get_generalized_velocity().CopyToVector();

  *residual << proposed_qdot[0] - state.theta1dot(),
               proposed_qdot[1] - state.theta2dot(),
               M * proposed_vdot - (B * tau - bias);
}

template <typename T>
T AcrobotPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const AcrobotState<T>& state = dynamic_cast<const AcrobotState<T>&>(
      context.get_continuous_state_vector());

  Matrix2<T> M = MassMatrix(context);
  Vector2<T> qdot(state.theta1dot(), state.theta2dot());

  return 0.5 * qdot.transpose() * M * qdot;
}

template <typename T>
T AcrobotPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const AcrobotState<T>& state = get_state(context);
  const AcrobotParams<T>& p = get_parameters(context);

  using std::cos;
  const T c1 = cos(state.theta1());
  const T c12 = cos(state.theta1() + state.theta2());

  return -p.m1() * p.gravity() * p.lc1() * c1 -
         p.m2() * p.gravity() * (p.l1() * c1 + p.lc2() * c12);
}

template <typename T>
AcrobotWEncoder<T>::AcrobotWEncoder(bool acrobot_state_as_second_output) {
  systems::DiagramBuilder<T> builder;

  acrobot_plant_ = builder.template AddSystem<AcrobotPlant<T>>();
  acrobot_plant_->set_name("acrobot_plant");
  auto encoder =
      builder.template AddSystem<systems::sensors::RotaryEncoders<T>>(
          4, std::vector<int>{0, 1});
  encoder->set_name("encoder");
  builder.Cascade(*acrobot_plant_, *encoder);
  builder.ExportInput(acrobot_plant_->get_input_port(0), "elbow_torque");
  builder.ExportOutput(encoder->get_output_port(), "measured_joint_positions");
  if (acrobot_state_as_second_output)
    builder.ExportOutput(acrobot_plant_->get_output_port(0), "acrobot_state");

  builder.BuildInto(this);
}

template <typename T>
AcrobotState<T>& AcrobotWEncoder<T>::get_mutable_acrobot_state(
    systems::Context<T>* context) const {
  AcrobotState<T>* x = dynamic_cast<AcrobotState<T>*>(
      &this->GetMutableSubsystemContext(*acrobot_plant_, context)
           .get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x != nullptr);
  return *x;
}

std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>& acrobot) {
  auto context = acrobot.CreateDefaultContext();

  // Set nominal torque to zero.
  acrobot.GetInputPort("elbow_torque").FixValue(context.get(), 0.0);

  // Set nominal state to the upright fixed point.
  AcrobotState<double>* x = dynamic_cast<AcrobotState<double>*>(
      &context->get_mutable_continuous_state_vector());
  DRAKE_ASSERT(x != nullptr);
  x->set_theta1(M_PI);
  x->set_theta2(0.0);
  x->set_theta1dot(0.0);
  x->set_theta2dot(0.0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  return systems::controllers::LinearQuadraticRegulator(acrobot, *context, Q,
                                                        R);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::examples::acrobot::AcrobotPlant)

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::drake::examples::acrobot::AcrobotWEncoder)
