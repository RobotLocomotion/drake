#include "drake/examples/cosserat_rod/cosserat_rod_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/multibody/multibody_tree/rod_element.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace cosserat_rod {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

using namespace multibody;

template <typename T>
CosseratRodPlant<T>::CosseratRodPlant(
    double length, double radius, double mass,
    double young_modulus, double shear_modulus,
    double tau_bending, double tau_twisting,
    int num_links) :
      length_(length),
      mass_(mass),
      tau_bending_(tau_bending), tau_twisting_(tau_twisting),
      num_elements_(num_links) {

  // Geometric parameters for a circular cross section:
  const double A = M_PI * radius * radius;
  const double I = A * A / (4 * M_PI);  // I1 = I2 = I, I3 = 2 * I.

  // Geometry functors:
  area_= [A](const T& s) -> T { return A; };
  moment_of_inertia1_ = [I](const T& s) -> T { return I; };
  moment_of_inertia2_ = [I](const T& s) -> T { return I; };
  moment_of_inertia3_ = [I](const T& s) -> T { return 2 * I; };
  young_modulus_ = [young_modulus](const T& s) -> T { return young_modulus; };
  shear_modulus_ = [shear_modulus](const T& s) -> T { return shear_modulus; };

  BuildMultibodyModeler();
  DRAKE_DEMAND(modeler_.get_num_positions() == 3 * num_links);
  DRAKE_DEMAND(modeler_.get_num_velocities() == 3 * num_links);
  DRAKE_DEMAND(modeler_.get_num_states() == 6 * num_links);

  //this->DeclareInputPort(systems::kVectorValued, 1);
  state_output_port_index_ = this->DeclareVectorOutputPort(
      systems::BasicVector<T>(modeler_.get_num_states()),
      &CosseratRodPlant::OutputState).get_index();
  this->DeclareContinuousState(
      modeler_.get_num_positions()  /* num_q */,
      modeler_.get_num_velocities() /* num_v */,
      0 /* num_z */);

  // Energy output port.
  energy_output_port_index_ = this->DeclareVectorOutputPort(
      systems::BasicVector<T>(2),
      [this](const systems::Context<T>& context,
             systems::BasicVector<T>* output) {
        output->SetAtIndex(0, this->DoCalcKineticEnergy(context));
        output->SetAtIndex(1, this->DoCalcPotentialEnergy(context));
      }).get_index();
}

#if 0
template <typename T>
template <typename U>
CosseratRodPlant<T>::CosseratRodPlant(
    const CosseratRodPlant<U>& other) : CosseratRodPlant<T>(
          other.length_,
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
#endif

template <typename T>
const RigidLink<T>& CosseratRodPlant<T>::AddLinkElement(
    int element_index, const multibody::Link<T>& parent, const T& s) {
  const double element_mass = mass() / num_elements_;
  const double element_length = length_ / num_elements_;

  // Rotational inertia about the material frame Q'com, Qcm, expressed in Q.
  // TODO: CREATE FACTORY IN UnitInertia FOR THIS SPECIAL CASE OF A PRISM OF
  // ARBITRARY CROSS SECTION SHAPE.
  const double Ixx = ExtractDoubleOrThrow(moment_of_inertia1_(s) / area_(s)) +
      element_length * element_length / 12.0;
  const double Iyy =
      ExtractDoubleOrThrow(moment_of_inertia2_(s) / area_(s)) +
          element_length * element_length / 12.0;
  const double Izz = ExtractDoubleOrThrow(moment_of_inertia3_(s) / area_(s));

  UnitInertia<double> G_Qcm(Ixx, Iyy, Izz);

  SpatialInertia<double> M_Qcm(element_mass, Vector3<double>::Zero(), G_Qcm);

  std::stringstream stream;
  stream << "Element_" << element_index;
  const RigidLink<T>& link_element =
      modeler_.template AddLink<RigidLink>(stream.str(), M_Qcm);

  // Pose of joint inboard frame F in the parent element frame Qim.
  Isometry3d X_QimF = Isometry3d::Identity();
  if (element_index != 0) {  // not the world.
    X_QimF = Translation3d(0.0, 0.0, element_length / 2.0);
  }

  // Pose of joint outboard frame M in this element's frame Qi.
  const Isometry3d X_QiM{Translation3d(0.0, 0.0, -element_length / 2.0)};

  stream.clear();
  stream << "Joint_" << element_index;
  const RevoluteJoint<T>& joint = modeler_.template AddJoint<RevoluteJoint>(
      stream.str(),
      parent, X_QimF, link_element, X_QiM, Vector3d::UnitX());
  joints_.push_back(&joint);

  // Add force element modelling the rod's internal forces.
  MultibodyTree<T>& model = modeler_.get_mutable_multibody_tree_model();

  const double B1 =
      ExtractDoubleOrThrow(young_modulus_(s) * moment_of_inertia1_(s));
  const double B2 =
      ExtractDoubleOrThrow(young_modulus_(s) * moment_of_inertia2_(s));
  const double C =
      ExtractDoubleOrThrow(shear_modulus_(s) * moment_of_inertia3_(s));

  // TODO: change to link_element.get_body() (with proper throw if model has
  // more than one body.
  model.template AddForceElement<RodElement>(
      modeler_.get_link_body(parent), element_length,
      modeler_.get_link_body(link_element), element_length,
      B1, B2, C, tau_bending_, tau_twisting_);

  return link_element;
}

template <typename T>
void CosseratRodPlant<T>::BuildMultibodyModeler() {
  const Link<T>& world = modeler_.get_world_link();

  double element_length = length_ / num_elements_;

  // First attached to the world.
  const RigidLink<T>* parent_element =
      &AddLinkElement(0, world, element_length / 2.0);

  // Add the rest.
  for (int element_index = 1; element_index < num_elements_; ++element_index) {
    const T s = element_index * element_length + element_length / 2.0;
    parent_element = &AddLinkElement(element_index, *parent_element, s);
  }
  DRAKE_ASSERT(static_cast<int>(joints_.size()) == num_elements_);

  modeler_.Finalize();
}

template <typename T>
std::unique_ptr<systems::LeafContext<T>>
CosseratRodPlant<T>::DoMakeContext() const {
  return modeler_.CreateDefaultContext();
}

template <typename T>
void CosseratRodPlant<T>::SetHorizontalCantileverState(
    systems::Context<T>* context) const {
  // The first joint, connecting to the world has angle = -pi/2:
  joints_[0]->set_angle(context, M_PI / 2);

  // Set the rest to zero angle.
  for (int joint_index(1); joint_index < num_elements_; ++joint_index) {
    const multibody::RevoluteJoint<T>* joint = joints_[joint_index];
    const double angle = 0.0;
    joint->set_angle(context, angle);
  }
}

template <typename T>
void CosseratRodPlant<T>::OutputState(
    const systems::Context<T>& context, 
    systems::BasicVector<T>* state_port_value) const {
  // Output port value is just the continuous or discrete state.
  const VectorX<T> state = context.get_continuous_state()->CopyToVector();
  state_port_value->SetFromVector(state);
}

// Compute the actual physics.
template <typename T>
void CosseratRodPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();

  const int nv = modeler_.get_num_velocities();

  MatrixX<T> M(nv, nv);
  modeler_.CalcMassMatrixViaInverseDynamics(context, M);

  VectorX<T> C(nv);
  modeler_.CalcBiasTerm(context, C);

  auto v = x.bottomRows(nv);

  // TODO: write MultibodyModeler::MapVelocityToQdot().
  VectorX<T> qdot(nv);
  qdot = v;

  VectorX<T> xdot(modeler_.get_num_states());
  xdot << qdot, M.llt().solve(- C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
T CosseratRodPlant<T>::DoCalcKineticEnergy(
    const systems::Context<T>& context) const {
#if 0
  const systems::BasicVector<T>& x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());

  const T theta1dot = x[2];
  const T theta2dot = x[3];

  Matrix2<T> H = MatrixH(context);
  Vector2<T> qdot(theta1dot, theta2dot);

  return 0.5 * qdot.transpose() * H * qdot;
#endif
  return 0.0;
}

template <typename T>
T CosseratRodPlant<T>::DoCalcPotentialEnergy(
    const systems::Context<T>& context) const {
#if 0
  const systems::BasicVector<T>& x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector());

  const T theta1 = x[0];
  const T theta2 = x[1];

  using std::cos;
  const T c1 = cos(theta1);
  const T c12 = cos(theta1 + theta2);

  return -m1_ * g_ * lc1_ * c1 - m2_ * g_ * (l1_ * c1 + lc2_ * c12);
#endif
  return 0.0;
}

template class CosseratRodPlant<double>;
template class CosseratRodPlant<AutoDiffXd>;

}  // namespace cosserat_rod
}  // namespace examples
}  // namespace drake
