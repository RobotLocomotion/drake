#include "drake/examples/acrobot/multibody/acrobot_multibody_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/multibody/benchmarks/acrobot/acrobot.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace acrobot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using drake::multibody::benchmarks::Acrobot;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

using namespace multibody;

template <typename T>
AcrobotMultibodyPlant<T>::AcrobotMultibodyPlant(double m1, double m2, double l1, double l2,
                              double lc1, double lc2, double Ic1, double Ic2,
                              double b1, double b2, double g)
    : systems::LeafSystem<T>(
          systems::SystemTypeTag<acrobot::AcrobotMultibodyPlant>{}),
      m1_(m1),
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
  BuildMultibodyModeler();
  DRAKE_DEMAND(modeler_.get_num_positions() == 2);
  DRAKE_DEMAND(modeler_.get_num_velocities() == 2);
  DRAKE_DEMAND(modeler_.get_num_states() == 4);

  this->DeclareInputPort(systems::kVectorValued, 1);
  this->DeclareVectorOutputPort(
      systems::BasicVector<T>(modeler_.get_num_states()),
      &AcrobotMultibodyPlant::OutputState);
  this->DeclareContinuousState(2 /* num_q */, 2 /* num_v */, 0 /* num_z */);

  // Energy output port.
  this->DeclareVectorOutputPort(
      systems::BasicVector<T>(2),
      [this](const systems::Context<T>& context,
             systems::BasicVector<T>* output) {
        output->SetAtIndex(0, this->DoCalcKineticEnergy(context));
        output->SetAtIndex(1, this->DoCalcPotentialEnergy(context));
      });
}

template <typename T>
template <typename U>
AcrobotMultibodyPlant<T>::AcrobotMultibodyPlant(
    const AcrobotMultibodyPlant<U>& other) : AcrobotMultibodyPlant<T>(
          other.m1(),
          other.m2(),
          other.l1(),
          other.l2(),
          other.lc1(),
          other.lc2(),
          other.Ic1(),
          other.Ic2(),
          other.b1(),
          other.b2(),
          other.g()) {}

template <typename T>
void AcrobotMultibodyPlant<T>::BuildMultibodyModeler() {
  // Rotational inertia of a thin rod along the y-axis.
  //UnitInertia<double> G_L1cm =
    //  UnitInertia<double>::StraightLine(Ic1(), Vector3<double>::UnitY());
  UnitInertia<double> G_L1cm(Ic1() /* Ixx */, 0.0 /* Iyy */, Ic1() /* Izz */);
  SpatialInertia<double> M_L1cm(m1(), Vector3<double>::Zero(), G_L1cm);

  // Rotational inertia of a thin rod along the y-axis.
  //UnitInertia<double> G_L2cm =
    //  UnitInertia<double>::StraightLine(Ic2(), Vector3<double>::UnitY());
  UnitInertia<double> G_L2cm(Ic2() /* Ixx */, 0.0 /* Iyy */, Ic2() /* Izz */);
  SpatialInertia<double> M_L2cm(m2(), Vector3<double>::Zero(), G_L2cm);

  const Link<T>& world = modeler_.get_world_link();
  link1_ = &modeler_.template AddLink<RigidLink>("Link1", M_L1cm);
  link2_ = &modeler_.template AddLink<RigidLink>("Link2", M_L2cm);

  // Pose of the shoulder outboard frame So in link1's frame L1.
  const Isometry3d X_L1So{Translation3d(0.0, lc1(), 0.0)};

  // Shoulder joint.
  shoulder_ =
      &modeler_.template AddJoint<RevoluteJoint>(
          "ShoulderJoint",
          world, Isometry3d::Identity(), *link1_, X_L1So, Vector3d::UnitZ());

  // Elbow joint.
  // Pose of the elbow inboard frame Ei in the frame L1 of link1.
  const Isometry3d X_L1Ei{Translation3d(0.0, -lc1(), 0.0)};
  // Pose of the elbow outboard frame Eo in the frame L2 of link2.
  const Isometry3d X_L2Eo{Translation3d(0.0, lc2(), 0.0)};
  elbow_ =
      &modeler_.template AddJoint<RevoluteJoint>(
          "ElbowJoint",
          *link1_, X_L1Ei, *link2_, X_L2Eo, Vector3d::UnitZ());
  modeler_.Finalize();
}

template <typename T>
std::unique_ptr<systems::LeafContext<T>>
AcrobotMultibodyPlant<T>::DoMakeContext() const {
  return modeler_.CreateDefaultContext();
}

template <typename T>
void AcrobotMultibodyPlant<T>::OutputState(
    const systems::Context<T>& context, 
    systems::BasicVector<T>* state_port_value) const {
  // Output port value is just the continuous or discrete state.
  const VectorX<T> state = context.get_continuous_state()->CopyToVector();
  state_port_value->SetFromVector(state);
}

template <typename T>
Matrix2<T> AcrobotMultibodyPlant<T>::MatrixH(
    const systems::Context<T>& context) const {
  Matrix2<T> H;
  modeler_.CalcMassMatrixViaInverseDynamics(context, H);
  return H;
}

template <typename T>
Vector2<T> AcrobotMultibodyPlant<T>::VectorC(
    const systems::Context<T>& context) const {
  Vector2<T> C;
  modeler_.CalcBiasTerm(context, C);

  const systems::BasicVector<T>& x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());

  const T theta1 = x[0];
  const T theta2 = x[1];
  const T s1 = sin(theta1);
  const T s12 = sin(theta1 + theta2);

  // Add in G terms.
  C(0) += g_ * m1_ * lc1_ * s1 + g_ * m2_ * (l1_ * s1 + lc2_ * s12);
  C(1) += g_ * m2_ * lc2_ * s12;

  // Damping terms.
  //C(0) += b1_ * theta1dot;
  //C(1) += b2_ * theta2dot;

  return C;
}

// Compute the actual physics.
template <typename T>
void AcrobotMultibodyPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const systems::BasicVector<T>& x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());
  const T& tau = this->EvalVectorInput(context, 0)->GetAtIndex(0);

  Matrix2<T> H = MatrixH(context);
  Vector2<T> C = VectorC(context);
  Vector2<T> B(0, 1);  // input matrix

  const T theta1dot = x[2];
  const T theta2dot = x[3];

  Vector4<T> xdot;
  xdot << theta1dot, theta2dot, H.inverse() * (B * tau - C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
T AcrobotMultibodyPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
  const systems::BasicVector<T>& x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());

  const T theta1dot = x[2];
  const T theta2dot = x[3];

  Matrix2<T> H = MatrixH(context);
  Vector2<T> qdot(theta1dot, theta2dot);

  return 0.5 * qdot.transpose() * H * qdot;
}

template <typename T>
T AcrobotMultibodyPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
  const systems::BasicVector<T>& x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());

  const T theta1 = x[0];
  const T theta2 = x[1];

  using std::cos;
  const T c1 = cos(theta1);
  const T c12 = cos(theta1 + theta2);

  return -m1_ * g_ * lc1_ * c1 - m2_ * g_ * (l1_ * c1 + lc2_ * c12);
}

template class AcrobotMultibodyPlant<double>;
template class AcrobotMultibodyPlant<AutoDiffXd>;

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
