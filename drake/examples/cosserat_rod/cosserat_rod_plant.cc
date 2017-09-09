#include "drake/examples/cosserat_rod/cosserat_rod_plant.h"

#include <cmath>
#include <fstream>
#include <vector>

#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/examples/cosserat_rod/rod_element.h"
#include "drake/multibody/multibody_tree/ball_mobilizer.h"
#include "drake/multibody/multibody_tree/fixed_offset_frame.h"
#include "drake/multibody/multibody_tree/revolute_mobilizer.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/rendering/drake_visualizer_client.h"
#include "drake/systems/rendering/pose_bundle.h"

using std::sin;
using std::cos;

namespace drake {
namespace examples {
namespace cosserat_rod {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;
using multibody::SpatialForce;

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

//#define PRINT_VAR(a) (void)a;
//#define PRINT_VARn(a) (void)a;

using namespace multibody;

template <typename T>
CosseratRodPlant<T>::CosseratRodPlant(
    double length, double radius, double mass,
    double young_modulus, double shear_modulus,
    double tau_bending, double tau_twisting,
    int num_links, int dimension) :
      dimension_(dimension),
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

  BuildMultibodyModel();
  const int nq = (dimension == 2) ? 1 : 4;
  const int nv = (dimension == 2) ? 1 : 3;
  const int nx = nq + nv;
  DRAKE_DEMAND(model_.get_num_positions() == nq * num_links);
  DRAKE_DEMAND(model_.get_num_velocities() == nv * num_links);
  DRAKE_DEMAND(model_.get_num_states() == nx * num_links);

  PRINT_VAR(model_.get_num_positions());
  PRINT_VAR(model_.get_num_velocities());

  //this->DeclareInputPort(systems::kVectorValued, 1);
  state_output_port_index_ = this->DeclareVectorOutputPort(
      systems::BasicVector<T>(model_.get_num_states()),
      &CosseratRodPlant::OutputState).get_index();
  this->DeclareContinuousState(
      model_.get_num_positions()  /* num_q */,
      model_.get_num_velocities() /* num_v */,
      0 /* num_z */);

  // Energy output port.
  energy_output_port_index_ = this->DeclareVectorOutputPort(
      systems::BasicVector<T>(2),
      [this](const systems::Context<T>& context,
             systems::BasicVector<T>* output) {
        output->SetAtIndex(0, this->DoCalcKineticEnergy(context));
        output->SetAtIndex(1, this->DoCalcPotentialEnergy(context));
      }).get_index();

  // poses output port.
  poses_output_port_ =
      &this->DeclareAbstractOutputPort(
          systems::rendering::PoseBundle<T>(num_elements_),
          &CosseratRodPlant::CalcElementPosesOutput);
}

template <typename T>
void CosseratRodPlant<T>::DoPublish(
    const systems::Context<T>& context,
    const std::vector<const systems::PublishEvent<T>*>&) const {
  std::vector<Isometry3<T>> poses;
  CalcElementPoses(context, &poses);

  std::ofstream fs;
  fs.open("poses.dat", std::fstream::out | std::fstream::app);

  for (auto& pose : poses) {
    fs << context.get_time() << " "
       << pose.translation().transpose() << std::endl;
  }

  fs.close();
}

template <typename T>
void CosseratRodPlant<T>::CalcElementPoses(
    const systems::Context<T>& context,
    std::vector<Isometry3<T>>* poses) const {
  DRAKE_DEMAND(poses != nullptr);

  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);

  // Skip the world.
  poses->clear();
  for (BodyNodeIndex node_index(0);
       node_index < model_.get_num_bodies(); ++node_index) {
    poses->push_back(pc.get_X_WB(node_index));
  }
}

template <typename T>
void CosseratRodPlant<T>::CalcElementPosesOutput(
    const systems::Context<T>& context,
    systems::rendering::PoseBundle<T>* bundle) const {
  DRAKE_DEMAND(bundle != nullptr);
  DRAKE_DEMAND(bundle->get_num_poses() == num_elements_);

  PositionKinematicsCache<T> pc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);

  // Skip the world.
  const int instance_id = 0;
  for (BodyNodeIndex node_index(1);
       node_index < model_.get_num_bodies(); ++node_index) {
    const int element_index = node_index - 1;
    std::stringstream stream;
    stream << "element_" << element_index;
    bundle->set_model_instance_id(element_index, instance_id);
    bundle->set_name(element_index, stream.str());
    Isometry3<T> X_DW = Isometry3<T>::Identity();
    X_DW.linear() =
        Eigen::AngleAxis<T>(M_PI/2, Vector3<T>::UnitX()).toRotationMatrix();
    bundle->set_pose(element_index, X_DW*pc.get_X_WB(node_index));
    PRINT_VAR(element_index);
    PRINT_VARn(pc.get_X_WB(node_index).matrix());
  }
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
const RigidBody<T>& CosseratRodPlant<T>::AddElement(
    int element_index, const multibody::Body<T>& element_im, const T& s) {
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

  // Create and add i-th element.
  const RigidBody<T>& element_i =
      model_.template AddBody<RigidBody>(M_Qcm);

  // Pose of joint inboard frame F in the element_im element_i frame Qim.
  Isometry3d X_QimF = Isometry3d::Identity();
  if (element_index != 0) {  // not the world.
    X_QimF = Translation3d(0.0, 0.0, element_length / 2.0);
  }
  const Frame<T>& inboard_frame_on_Qim =
      model_.template AddFrame<FixedOffsetFrame>(element_im, X_QimF);

  // Pose of joint outboard frame M in this element_i's frame Qi.
  const Isometry3d X_QiM{Translation3d(0.0, 0.0, -element_length / 2.0)};
  const Frame<T>& outboard_frame_on_Qi =
      model_.template AddFrame<FixedOffsetFrame>(element_i, X_QiM);

  stream.clear();
  stream << "Joint_" << element_index;
  if (dimension_ == 2) {
    const RevoluteMobilizer<T> &mobilizer =
        model_.template AddMobilizer<RevoluteMobilizer>(
            inboard_frame_on_Qim, outboard_frame_on_Qi, Vector3d::UnitX());
    mobilizers_.push_back(&mobilizer);
  } else {
    const BallMobilizer<T>& mobilizer =
        model_.template AddMobilizer<BallMobilizer>(
            inboard_frame_on_Qim, outboard_frame_on_Qi);
    mobilizers_.push_back(&mobilizer);
  }

  const double B1 =
      ExtractDoubleOrThrow(young_modulus_(s) * moment_of_inertia1_(s));
  const double B2 =
      ExtractDoubleOrThrow(young_modulus_(s) * moment_of_inertia2_(s));
  const double C =
      ExtractDoubleOrThrow(shear_modulus_(s) * moment_of_inertia3_(s));

  double element_im_length = element_length;
  if (element_index == 0) element_im_length = 0.0;  // the world, BC.
#if 0
  model_.template AddForceElement<RodElement>(
      element_im, element_im_length,
      element_i, element_length,
      B1, B2, C, tau_bending_, tau_twisting_);
#endif
  (void) B1;
  (void) B2;
  (void) C;

  return element_i;
}

template <typename T>
void CosseratRodPlant<T>::BuildMultibodyModel() {
  const Body<T>& world = model_.get_world_body();

  double element_length = length_ / num_elements_;

  // First element is attached to the world.
  first_element_ = &AddElement(0, world, element_length / 2.0);


  // Add the rest.
  const RigidBody<T>* parent_element = first_element_;
  for (int element_index = 1; element_index < num_elements_; ++element_index) {
    const T s = element_index * element_length + element_length / 2.0;
    parent_element = &AddElement(element_index, *parent_element, s);
  }
  DRAKE_ASSERT(static_cast<int>(mobilizers_.size()) == num_elements_);
  last_element_ = parent_element;

  model_.template AddForceElement<UniformGravityFieldElement>(
      Vector3<double>(0.0, -9.81, 0.0));

  model_.Finalize();
}

template <typename T>
std::unique_ptr<systems::LeafContext<T>>
CosseratRodPlant<T>::DoMakeContext() const {
  return model_.CreateDefaultContext();
}

template <typename T>
void CosseratRodPlant<T>::SetHorizontalCantileverState(
    systems::Context<T>* context) const {

  model_.SetDefaults(context);

#if 0
  // The first joint, connecting to the world has angle = -pi/2:
  //mobilizers_[0]->set_angle(context, M_PI / 2);
  // Change these boundary conditions when there is a WeldMobilizer.
  mobilizers_[0]->set_angle(context, 0.0);

  // Set the rest to zero angle.
  for (int joint_index(1); joint_index < num_elements_; ++joint_index) {
    const multibody::RevoluteMobilizer<T>* mobilizer = mobilizers_[joint_index];
    const double angle = 0.0;
    mobilizer->set_angle(context, angle);
    mobilizer->set_angular_rate(context, 0.0);
  }
#endif
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

  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  PRINT_VAR("CosseratRodPlant<T>::DoCalcTimeDerivatives");
  PRINT_VAR("CosseratRodPlant<T>::DoCalcTimeDerivatives");

  PositionKinematicsCache<T> pc(model_.get_topology());
  VelocityKinematicsCache<T> vc(model_.get_topology());
  model_.CalcPositionKinematicsCache(context, &pc);
  model_.CalcVelocityKinematicsCache(context, pc, &vc);

  // Add a constant moment at the end link.
  std::vector<SpatialForce<T>> Fapplied_Bo_W_array(model_.get_num_bodies());
  for (auto& F : Fapplied_Bo_W_array) F.SetZero();

  PRINT_VAR(last_element_->get_node_index());

  MatrixX<T> M(nv, nv);
  model_.CalcMassMatrixViaInverseDynamics(context, pc, &M);

  PRINT_VARn(M);

  VectorX<T> tau = VectorX<T>::Zero(nv);
  model_.CalcForceElementsContribution(
      context, pc, vc,
      &Fapplied_Bo_W_array, &tau);

  // Add a constant moment at the end link.
  const T M0 = 0.0;  // Torque in Nm
  SpatialForce<T> M_W(Vector3<T>(M0, 0.0, 0.0), Vector3<T>::Zero());
  Fapplied_Bo_W_array[last_element_->get_node_index()] += M_W;

  PRINT_VAR(tau.transpose());
  for (auto& F : Fapplied_Bo_W_array) {
    PRINT_VAR(F);
  }

  VectorX<T> C(nv);
  model_.CalcBiasTerm(context, pc, vc, Fapplied_Bo_W_array, &C);

  PRINT_VAR(C.transpose());

  auto v = x.bottomRows(nv);

  VectorX<T> qdot(nq);
  model_.MapVelocityToQDot(context, v, &qdot);

  VectorX<T> xdot(model_.get_num_states());

  // Check if M is symmetric.
  const T err_sym = (M - M.transpose()).norm();
  PRINT_VAR(err_sym);
  //DRAKE_DEMAND(err_sym < 1.0e-6);

  // TODO: this solve M.llt().solve() does not seem to throw an error when M is
  // not symmetric. Apparently my M is not symmetric when I add those ball
  // mobilizers.
  //Eigen::LLT<MatrixX<T>> solver(M);

  xdot << qdot, M.llt().solve(- C);
  derivatives->SetFromVector(xdot);
}

template <typename T>
void CosseratRodPlant<T>::DoMapVelocityToQDot(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& generalized_velocity,
    systems::VectorBase<T>* positions_derivative) const {
  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  DRAKE_DEMAND(positions_derivative != nullptr);
  DRAKE_DEMAND(positions_derivative->size() == nq);
  DRAKE_ASSERT(generalized_velocity.size() == nv);

  VectorX<T> qdot(nq);
  model_.MapVelocityToQDot(context, generalized_velocity, &qdot);
  positions_derivative->SetFromVector(qdot);
}

template <typename T>
void CosseratRodPlant<T>::DoMapQDotToVelocity(
    const systems::Context<T>& context,
    const Eigen::Ref<const VectorX<T>>& configuration_dot,
    systems::VectorBase<T>* generalized_velocity) const {
  const int nq = model_.get_num_positions();
  const int nv = model_.get_num_velocities();

  DRAKE_DEMAND(generalized_velocity != nullptr);
  DRAKE_DEMAND(generalized_velocity->size() == nv);
  DRAKE_ASSERT(configuration_dot.size() == nq);

  VectorX<T> v(nv);
  model_.MapQDotToVelocity(context, configuration_dot, &v);
  generalized_velocity->SetFromVector(v);
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

template <typename T>
void CosseratRodPlant<T>::DoProjectQ(systems::Context<T>* context) const {
  PRINT_VAR("CosseratRodPlant::DoProjectQ()");
  PRINT_VAR(model_.get_num_positions());
  PRINT_VAR(model_.get_num_velocities());

  if (dimension_ == 2) return;

  for (auto mobilizer : mobilizers_) {
    const BallMobilizer<T>* ball =
        dynamic_cast<const BallMobilizer<T>*>(mobilizer);
    Quaternion<T> q = ball->get_quaternion(*context);
    ball->set_quaternion(context, q.normalized());
    PRINT_VAR(q.norm());
    PRINT_VAR(ball->get_quaternion(*context).norm());
  }
}

template <typename T>
void CosseratRodPlant<T>::MakeViewerLoadMessage(
    drake::lcmt_viewer_load_robot* message) const {
  DRAKE_DEMAND(message != nullptr);

  // Compute a radius for an equivalent cylindrical shape.
  const double A = ExtractDoubleOrThrow(area_(0.0));
  const double radius = std::sqrt(A / M_PI);
  const double element_length = length_ / num_elements_;

  // Create the rod visualization.
  DrakeShapes::VisualElement element_vis(
      DrakeShapes::Cylinder(radius, element_length),
      Eigen::Isometry3d::Identity(), Eigen::Vector4d(0.7, 0.7, 0.7, 1));

  // Create the load message.
  message->num_links = num_elements_;
  message->link.resize(num_elements_);
  for (int ielement = 0; ielement < num_elements_; ++ielement) {
    std::stringstream stream;
    stream << "CosseratRodElements::element_" << ielement;
    message->link[ielement].name = stream.str();
    message->link[ielement].robot_num = 0;
    message->link[ielement].num_geom = 1;
    message->link[ielement].geom.resize(1);
    message->link[ielement].geom[0] =
        drake::systems::rendering::MakeGeometryData(element_vis);
  }
}

template class CosseratRodPlant<double>;
template class CosseratRodPlant<AutoDiffXd>;

}  // namespace cosserat_rod
}  // namespace examples
}  // namespace drake
