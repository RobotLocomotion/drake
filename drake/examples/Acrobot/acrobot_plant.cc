#include "drake/examples/Acrobot/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/sensors/rotary_encoders.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace acrobot {

namespace {
constexpr int kNumDOF = 2;  // theta1 + theta2.
}

template <typename T>
AcrobotPlant<T>::AcrobotPlant(double m1, double m2, double l1, double l2,
                              double lc1, double lc2, double Ic1, double Ic2,
                              double b1, double b2, double g)
    : m1_(m1),
      m2_(m2),
      l1_(l1),
      l2_(l2),
      lc1_(lc1),
      lc2_(lc2),
      Ic1_(Ic1),
      Ic2_(Ic2),
      b1_(b1),
      b2_(b2),
      g_(g) {
  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareVectorOutputPort(AcrobotStateVector<T>());
  static_assert(AcrobotStateVectorIndices::kNumCoordinates == kNumDOF * 2, "");
  this->DeclareContinuousState(
      AcrobotStateVector<T>(),
      kNumDOF /* num_q */,
      kNumDOF /* num_v */,
      0 /* num_z */);
}

template <typename T>
std::unique_ptr<AcrobotPlant<T>> AcrobotPlant<T>::CreateAcrobotMIT() {
  return std::make_unique<AcrobotPlant<T>>(2.4367,   // m1
                                           0.6178,   // m2
                                           0.2563,   // l1
                                           0,        // l2
                                           1.6738,   // lc1
                                           1.5651,   // lc2
                                           -4.7443,  // Ic1
                                           -1.0068,  // Ic2
                                           0.0320,   // b1
                                           0.0413);  // b2
  // Parameters are identified in a way that torque has the unit of current
  // (Amps), in order to simplify the implementation of torque constraint on
  // motors. Therefore, numbers here do not carry physical meanings.
}

template <typename T>
void AcrobotPlant<T>::DoCalcOutput(const systems::Context<T>& context,
                                   systems::SystemOutput<T>* output) const {
  output->GetMutableVectorData(0)->set_value(
      dynamic_cast<const AcrobotStateVector<T>&>(
          context.get_continuous_state_vector())
          .get_value());
}

template <typename T>
Matrix2<T> AcrobotPlant<T>::MatrixH(const AcrobotStateVector<T>& x) const {
  const T c2 = cos(x.theta2());

  const T h12 = I2_ + m2l1lc2_ * c2;
  Matrix2<T> H;
  H << I1_ + I2_ + m2_ * l1_ * l1_ + 2 * m2l1lc2_ * c2, h12, h12, I2_;
  return H;
}

template <typename T>
Vector2<T> AcrobotPlant<T>::VectorC(const AcrobotStateVector<T>& x) const {
  const T s1 = sin(x.theta1()), s2 = sin(x.theta2());
  const T s12 = sin(x.theta1() + x.theta2());

  Vector2<T> C;
  C << -2 * m2l1lc2_ * s2 * x.theta2dot() * x.theta1dot() +
           -m2l1lc2_ * s2 * x.theta2dot() * x.theta2dot(),
      m2l1lc2_ * s2 * x.theta1dot() * x.theta1dot();

  // Add in G terms.
  C(0) += g_ * m1_ * lc1_ * s1 + g_ * m2_ * (l1_ * s1 + lc2_ * s12);
  C(1) += g_ * m2_ * lc2_ * s12;

  // Damping terms.
  C(0) += b1_ * x.theta1dot();
  C(1) += b2_ * x.theta2dot();

  return C;
}

// Compute the actual physics.
template <typename T>
void AcrobotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());
  const T& tau = this->EvalVectorInput(context, 0)->GetAtIndex(0);

  Matrix2<T> H = MatrixH(x);
  Vector2<T> C = VectorC(x);
  Vector2<T> B(0, 1);  // input matrix

  Vector4<T> xdot;
  xdot << x.theta1dot(), x.theta2dot(), H.inverse() * (B * tau - C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
T AcrobotPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());

  Matrix2<T> H = MatrixH(x);
  Vector2<T> qdot(x.theta1dot(), x.theta2dot());

  return 0.5 * qdot.transpose() * H * qdot;
}

template <typename T>
T AcrobotPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());

  using std::cos;
  const T c1 = cos(x.theta1());
  const T c12 = cos(x.theta1() + x.theta2());

  return -m1_ * g_ * lc1_ * c1 - m2_ * g_ * (l1_ * c1 + lc2_ * c12);
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
  builder.ExportOutput(encoder->get_output_port());
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
    const AcrobotPlant<double>& acrobot) {
  auto context = acrobot.CreateDefaultContext();

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

  return systems::LinearQuadraticRegulator(acrobot, *context, Q, R);
}

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
