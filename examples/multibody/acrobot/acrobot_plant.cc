#include "drake/examples/multibody/acrobot/acrobot_plant.h"

#include <cmath>
#include <vector>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_throw.h"
#include "drake/common/eigen_autodiff_types.h"
#include "drake/geometry/frame_id_vector.h"
#include "drake/geometry/frame_kinematics_vector.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"
#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::FrameId;
using geometry::FrameIdVector;
using geometry::FramePoseVector;
using geometry::GeometryFrame;
using geometry::GeometryInstance;
using geometry::GeometrySystem;
using geometry::SourceId;
using geometry::Sphere;
using drake::multibody::BodyIndex;
using drake::multibody::MultibodyTree;
using drake::multibody::MultibodyTreeContext;
using drake::multibody::MultibodyForces;
using drake::multibody::PositionKinematicsCache;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialAcceleration;
using drake::multibody::SpatialForce;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using drake::multibody::VelocityKinematicsCache;
using systems::BasicVector;
using systems::Context;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::State;

template<typename T>
AcrobotPlant<T>::AcrobotPlant(
    double m1, double m2, double l1, double l2,
    double lc1, double lc2, double Ic1, double Ic2,
    double b1, double b2, double g) :
    systems::LeafSystem<T>(systems::SystemTypeTag<
        drake::examples::multibody::acrobot::AcrobotPlant>()),
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
  BuildMultibodyTreeModel();

  // Some very basic verification that the model is what we expect it to be.
  DRAKE_DEMAND(model_->get_num_positions() == 2);
  DRAKE_DEMAND(model_->get_num_velocities() == 2);
  DRAKE_DEMAND(model_->get_num_states() == 4);

  this->DeclareContinuousState(
      BasicVector<T>(model_->get_num_states()),
      model_->get_num_positions(),
      model_->get_num_velocities(), 0 /* num_z */);

  // Declare a vector input of size one for an applied torque at the
  // elbow joint.
  applied_torque_input_ =
      this->DeclareVectorInputPort(BasicVector<T>(1)).get_index();

  // Declare a port that outputs the state.
  state_output_port_ = this->DeclareVectorOutputPort(
      AcrobotState<T>(), &AcrobotPlant::CopyStateOut).get_index();
}

template<typename T>
AcrobotPlant<T>::AcrobotPlant(
    geometry::GeometrySystem<double>* geometry_system) : AcrobotPlant() {
  DRAKE_DEMAND(geometry_system != nullptr);
  RegisterGeometry(geometry_system);
  DeclareGeometrySystemPorts();
}

template<typename T>
template<typename U>
AcrobotPlant<T>::AcrobotPlant(
    const AcrobotPlant<U> &other) :
    AcrobotPlant(other.m1(), other.m2(),
                 other.l1(), other.l2(),
                 other.lc1(), other.lc2(),
                 other.Ic1(), other.Ic2(),
                 other.b1(), other.b2(), other.g()) {
  source_id_ = other.source_id_;
  link1_frame_id_ = other.link1_frame_id_;
  link2_frame_id_ = other.link2_frame_id_;

  // Only declare ports to communicate with a GeometrySystem if the plant is
  // provided with a valid source id.
  if (source_id_) DeclareGeometrySystemPorts();
}

template<typename T>
void AcrobotPlant<T>::DeclareGeometrySystemPorts() {
  geometry_id_port_ =
      this->DeclareAbstractOutputPort(
          &AcrobotPlant::AllocateFrameIdOutput,
          &AcrobotPlant::CalcFrameIdOutput).get_index();
  geometry_pose_port_ =
      this->DeclareAbstractOutputPort(
          &AcrobotPlant::AllocateFramePoseOutput,
          &AcrobotPlant::CalcFramePoseOutput).get_index();
}

template <typename T>
FrameIdVector AcrobotPlant<T>::AllocateFrameIdOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  DRAKE_DEMAND(link1_frame_id_ != nullopt);
  DRAKE_DEMAND(link2_frame_id_ != nullopt);
  FrameIdVector ids(source_id_.value());
  // Add a frame for the one single body in this model.
  ids.AddFrameId(link1_frame_id_.value());
  ids.AddFrameId(link2_frame_id_.value());
  return ids;
}

template <typename T>
void AcrobotPlant<T>::CalcFrameIdOutput(
    const Context<T>&, FrameIdVector*) const {
  // Just a sanity check.
  DRAKE_DEMAND(source_id_ != nullopt);
  DRAKE_DEMAND(link1_frame_id_ != nullopt);
  DRAKE_DEMAND(link2_frame_id_ != nullopt);
  // NOTE: This only needs to do work if the topology changes. This system makes
  // no topology changes.
}

template <typename T>
FramePoseVector<T> AcrobotPlant<T>::AllocateFramePoseOutput(
    const Context<T>&) const {
  DRAKE_DEMAND(source_id_ != nullopt);
  FramePoseVector<T> poses(source_id_.value());
  // There are two moveable frames, one for each link.
  poses.mutable_vector().resize(2);
  return poses;
}

template <typename T>
void AcrobotPlant<T>::CalcFramePoseOutput(
    const Context<T>& context, FramePoseVector<T>* poses) const {
  DRAKE_ASSERT(static_cast<int>(poses->vector().size()) == 2);
  DRAKE_ASSERT(model_->get_num_bodies() == 3);  // Includes the world body.

  PositionKinematicsCache<T> pc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);

  std::vector<Isometry3<T>>& pose_data = poses->mutable_vector();
  // TODO(amcastro-tri): Make use of Body::EvalPoseInWorld(context) once caching
  // lands.
  pose_data[0] = pc.get_X_WB(link1_->get_node_index());
  pose_data[1] = pc.get_X_WB(link2_->get_node_index());
}

template <typename T>
void AcrobotPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                    AcrobotState<T>* output) const {
  output->set_theta1(shoulder_->get_angle(context));
  output->set_theta1dot(shoulder_->get_angular_rate(context));
  output->set_theta2(elbow_->get_angle(context));
  output->set_theta2dot(elbow_->get_angular_rate(context));
}

template <typename T>
const OutputPort<T>& AcrobotPlant<T>::get_geometry_id_output_port()
const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& AcrobotPlant<T>::get_geometry_pose_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const InputPortDescriptor<T>& AcrobotPlant<T>::get_input_port() const {
  return systems::System<T>::get_input_port(applied_torque_input_);
}

template <typename T>
const OutputPort<T>& AcrobotPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_output_port_);
}

template<typename T>
void AcrobotPlant<T>::BuildMultibodyTreeModel() {
  model_ = std::make_unique<MultibodyTree<T>>();
  DRAKE_DEMAND(model_ != nullptr);

  // COM's positions in each link (L1/L2) frame:
  // Frame L1's origin is located at the shoulder outboard frame.
  const Vector3d p_L1L1cm = -lc1() * Vector3d::UnitZ();
  // Frame L2's origin is located at the elbow outboard frame.
  const Vector3d p_L2L2cm = -lc2() * Vector3d::UnitZ();

  // Define each link's spatial inertia about their respective COM.
  RotationalInertia<double> G1_Bcm =
      UnitInertia<double>::StraightLine(Ic1(), Vector3d::UnitY());
  SpatialInertia<double> M1_L1o =
      SpatialInertia<double>::MakeFromCentralInertia(m1(), p_L1L1cm, G1_Bcm);

  UnitInertia<double> G2_Bcm =
      UnitInertia<double>::StraightLine(Ic2(), Vector3d::UnitY());
  SpatialInertia<double> M2_L2o =
      SpatialInertia<double>::MakeFromCentralInertia(m2(), p_L2L2cm, G2_Bcm);

  // Add a rigid body to model each link.
  link1_ = &model_->template AddBody<RigidBody>(M1_L1o);
  link2_ = &model_->template AddBody<RigidBody>(M2_L2o);

  shoulder_ = &model_->template AddJoint<RevoluteJoint>(
      "ShoulderJoint",
      /* Shoulder inboard frame Si IS the the world frame W. */
      model_->get_world_body(), {},
      /* Shoulder outboard frame So IS frame L1. */
      *link1_, {},
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  elbow_ = &model_->template AddJoint<RevoluteJoint>(
      "ElbowJoint",
      *link1_,
      /* Pose of the elbow inboard frame Ei in Link 1's frame. */
      Isometry3d(Translation3d(-l1() * Vector3d::UnitZ())),
      *link2_,
      /* Elbow outboard frame Eo IS frame L2 for link 2. */
      {},
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  // Gravity acting in the -z direction.
  model_->template AddForceElement<UniformGravityFieldElement>(
      -g() * Vector3d::UnitZ());

  model_->Finalize();
}

template<typename T>
void AcrobotPlant<T>::RegisterGeometry(
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  // Register this system to be a source for the input geometry_system.
  source_id_ = geometry_system->RegisterSource("multibody_acrobot");

  link1_frame_id_ = geometry_system->RegisterFrame(
      source_id_.value(),
      GeometryFrame(
          "Link1Frame",
          /* Initial pose of L2 in the world frame W. */
          Isometry3<double>::Identity()));

  link2_frame_id_ = geometry_system->RegisterFrame(
      source_id_.value(),
      GeometryFrame(
          "Link2Frame",
          /* Initial pose of L2 in the world frame W. */
          Isometry3d(Translation3d(-l1() * Vector3d::UnitZ()))));

  // A sphere at the world's origin (actually we are making it move with L1).
  geometry_system->RegisterGeometry(
      source_id_.value(), link1_frame_id_.value(),
      std::make_unique<GeometryInstance>(
          Isometry3d::Identity(), /* Geometry pose in link's frame */
          std::make_unique<Sphere>(l1() / 8)));

  // A rod geometry for link 1.
  const double rod1_radius = 0.05;
  geometry_system->RegisterGeometry(
      source_id_.value(), link1_frame_id_.value(),
      std::make_unique<GeometryInstance>(
          Isometry3d{Translation3d(-l1() / 2.0 * Vector3d::UnitZ())},
          std::make_unique<Cylinder>(rod1_radius, l1())));

  // A rod geometry for link 2.
  const double rod2_radius = 0.05;
  geometry_system->RegisterGeometry(
      source_id_.value(), link2_frame_id_.value(),
      std::make_unique<GeometryInstance>(
          Isometry3d{Translation3d(-l2() / 2.0 * Vector3d::UnitZ())},
          std::make_unique<Cylinder>(rod2_radius, l2())));
}

template<typename T>
std::unique_ptr<systems::LeafContext<T>>
AcrobotPlant<T>::DoMakeContext() const {
  return std::make_unique<MultibodyTreeContext<T>>(model_->get_topology());
}

template<typename T>
void AcrobotPlant<T>::DoCalcTimeDerivatives(
    const systems::Context<T>& context,
    systems::ContinuousState<T>* derivatives) const {
  const auto x =
      dynamic_cast<const systems::BasicVector<T>&>(
          context.get_continuous_state_vector()).get_value();
  const int nv = model_->get_num_velocities();

  // Mass matrix:
  MatrixX<T> M(nv, nv);
  // Forces:
  MultibodyForces<T> forces(*model_);
  // Bodies' accelerations, ordered by BodyNodeIndex.
  std::vector<SpatialAcceleration<T>> A_WB_array(model_->get_num_bodies());
  // Generalized accelerations:
  VectorX<T> vdot = VectorX<T>::Zero(nv);

  // TODO(amcastro-tri): Eval() these from the context.
  PositionKinematicsCache<T> pc(model_->get_topology());
  VelocityKinematicsCache<T> vc(model_->get_topology());
  model_->CalcPositionKinematicsCache(context, &pc);
  model_->CalcVelocityKinematicsCache(context, pc, &vc);

  // Compute forces applied through force elements. This effectively resets
  // the forces to zero and adds in contributions due to force elements:
  model_->CalcForceElementsContribution(context, pc, vc, &forces);

  // TODO(amcastro-tri): Add in input forces when #7797 lands.
  // Like so: elbow_->AddInTorque(context, get_tau(context), &forces);

  model_->CalcMassMatrixViaInverseDynamics(context, &M);

  // WARNING: to reduce memory foot-print, we use the input applied arrays also
  // as output arrays. This means that both Fapplied_Bo_W_array and tau_applied
  // get overwritten on output. This is not important in this case since we
  // don't need their values anymore. Please see the documentation for
  // CalcInverseDynamics() for details.
  // With vdot = 0, this computes:
  //   tau = C(q, v)v - tau_app - ∑ J_WBᵀ(q) Fapp_Bo_W.
  std::vector<SpatialForce<T>>& F_BBo_W_array = forces.mutable_body_forces();
  VectorX<T>& tau_array = forces.mutable_generalized_forces();
  model_->CalcInverseDynamics(
      context, pc, vc, vdot,
      F_BBo_W_array, tau_array,
      &A_WB_array,
      &F_BBo_W_array, /* Notice these arrays gets overwritten on output. */
      &tau_array);

  vdot = M.ldlt().solve(-tau_array);

  auto v = x.bottomRows(nv);
  VectorX<T> xdot(model_->get_num_states());
  // For this simple model v = qdot.
  xdot << v, vdot;
  derivatives->SetFromVector(xdot);
}

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class drake::examples::multibody::acrobot::AcrobotPlant)
