#include "drake/examples/Acrobot/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rotary_encoders.h"

namespace drake {
namespace examples {
namespace acrobot {

namespace {
constexpr int kNumDOF = 2;  // theta1 + theta2.
}

template <typename T>
AcrobotPlant<T>::AcrobotPlant() {
  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareContinuousState(kNumDOF * 2);  // Position + velocity.
  this->DeclareOutputPort(systems::kVectorValued, kNumDOF * 2);
}

template <typename T>
void AcrobotPlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                 systems::SystemOutput<T>* output) const {
  output->GetMutableVectorData(0)->set_value(
      dynamic_cast<const AcrobotStateVector<T>&>(
          context.get_continuous_state_vector())
          .get_value());
}

// Compute the actual physics.
template <typename T>
void AcrobotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());
  const T& tau = this->EvalVectorInput(context, 0)->GetAtIndex(0);

  const double I1 = Ic1 + m1 * lc1 * lc1;
  const double I2 = Ic2 + m2 * lc2 * lc2;
  const double m2l1lc2 = m2 * l1 * lc2;  // occurs often!

  using std::sin;
  using std::cos;
  const T c2 = cos(x.theta2());
  const T s1 = sin(x.theta1()), s2 = sin(x.theta2());
  const T s12 = sin(x.theta1() + x.theta2());

  const T h12 = I2 + m2l1lc2 * c2;
  Eigen::Matrix<T, 2, 2> H;
  H << I1 + I2 + m2 * l1 * l1 + 2 * m2l1lc2 * c2, h12, h12, I2;

  Eigen::Matrix<T, 2, 1> C;
  C << -2 * m2l1lc2 * s2 * x.theta2dot() * x.theta1dot() +
           -m2l1lc2 * s2 * x.theta2dot() * x.theta2dot(),
      m2l1lc2 * s2 * x.theta1dot() * x.theta1dot();

  // add in G terms
  C(0) += g * m1 * lc1 * s1 + g * m2 * (l1 * s1 + lc2 * s12);
  C(1) += g * m2 * lc2 * s12;

  // damping terms
  C(0) += b1 * x.theta1dot();
  C(1) += b2 * x.theta2dot();

  // input matrix
  Eigen::Matrix<T, 2, 1> B;
  B << 0.0, 1.0;

  Eigen::Matrix<T, 4, 1> xdot;
  xdot << x.theta1dot(), x.theta2dot(), H.inverse() * (B * tau - C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
std::unique_ptr<systems::ContinuousState<T>>
AcrobotPlant<T>::AllocateContinuousState() const {
  return std::make_unique<systems::ContinuousState<T>>(
      std::make_unique<AcrobotStateVector<T>>(), kNumDOF /* num_q */,
      kNumDOF /* num_v */, 0 /* num_z */);
}

template <typename T>
std::unique_ptr<systems::BasicVector<T>> AcrobotPlant<T>::AllocateOutputVector(
    const systems::SystemPortDescriptor<T>& descriptor) const {
  DRAKE_THROW_UNLESS(descriptor.get_size() == kNumDOF * 2);
  return std::make_unique<AcrobotStateVector<T>>();
}

// AcrobotPlant has no constructor arguments, so there's no work to do here.
template <typename T>
AcrobotPlant<AutoDiffXd>* AcrobotPlant<T>::DoToAutoDiffXd() const {
  return new AcrobotPlant<AutoDiffXd>();
}

template class AcrobotPlant<double>;
template class AcrobotPlant<AutoDiffXd>;

template <typename T>
AcrobotWEncoder<T>::AcrobotWEncoder(bool acrobot_state_as_second_output) {
  systems::DiagramBuilder<T> builder;

  acrobot_plant_ = builder.template AddSystem<AcrobotPlant<T>>();
  auto encoder =
      builder.template AddSystem<systems::sensors::RotaryEncoders<T>>(
          4, std::vector<int>{0, 1});
  builder.Cascade(*acrobot_plant_, *encoder);
  builder.ExportInput(acrobot_plant_->get_input_port(0));
  builder.ExportOutput(encoder->get_output_port(0));
  if (acrobot_state_as_second_output)
    builder.ExportOutput(acrobot_plant_->get_output_port(0));

  builder.BuildInto(this);
}

template <typename T>
AcrobotStateVector<T>* AcrobotWEncoder<T>::get_mutable_acrobot_state(
    systems::Context<T>* context) const {
  AcrobotStateVector<T>* x = dynamic_cast<AcrobotStateVector<T>*>(
      this->GetMutableSubsystemContext(context, acrobot_plant_)
          ->get_mutable_continuous_state_vector());
  DRAKE_DEMAND(x != nullptr);
  return x;
}

template class AcrobotWEncoder<double>;
template class AcrobotWEncoder<AutoDiffXd>;

std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotPlant<double>* acrobot) {
  auto context = acrobot->CreateDefaultContext();

  // Set nominal torque to zero.
  context->FixInputPort(0, Vector1d::Constant(0.0));

  // Set nominal state to the upright fixed point.
  AcrobotStateVector<double>* x = dynamic_cast<AcrobotStateVector<double>*>(
      context->get_mutable_continuous_state_vector());
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

  return systems::LinearQuadraticRegulator(*acrobot, *context, Q, R);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
