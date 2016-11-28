#include "drake/examples/Acrobot/acrobot_plant.h"

#include <cmath>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/examples/Acrobot/gen/acrobot_state_vector.h"

namespace drake {
namespace examples {
namespace acrobot {

namespace {
constexpr int kNumDOF = 2;  // theta1 + theta2.
}

template <typename T>
AcrobotPlant<T>::AcrobotPlant() {
  this->DeclareInputPort(systems::kVectorValued, 1,
                         systems::kContinuousSampling);
  this->DeclareContinuousState(kNumDOF * 2);  // Position + velocity.
  this->DeclareOutputPort(systems::kVectorValued, kNumDOF * 2,
                          systems::kContinuousSampling);
}

template <typename T>
void AcrobotPlant<T>::EvalOutput(const systems::Context<T>& context,
                                 systems::SystemOutput<T>* output) const {
  output->GetMutableVectorData(0)->set_value(
      dynamic_cast<const AcrobotStateVector<T>&>(
          context.get_continuous_state_vector())
          .get_value());
}

// Compute the actual physics.
template <typename T>
void AcrobotPlant<T>::EvalTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  DRAKE_ASSERT_VOID(systems::System<T>::CheckValidContext(context));

  const AcrobotStateVector<T>& x = dynamic_cast<const AcrobotStateVector<T>&>(
      context.get_continuous_state_vector());
  const T tau = this->EvalVectorInput(context, 0)->GetAtIndex(0);

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

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
