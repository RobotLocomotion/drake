#include "drake/examples/multibody/acrobot/make_acrobot_plant.h"

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
using drake::multibody::multibody_plant::MultibodyPlant;
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
using systems::DiagramBuilder;
using systems::InputPortDescriptor;
using systems::OutputPort;
using systems::State;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeAcrobotPlant(
    const AcrobotParameters& params,
    geometry::GeometrySystem<double>* geometry_system) {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  // COM's positions in each link (L1/L2) frame:
  // Frame L1's origin is located at the shoulder outboard frame.
  const Vector3d p_L1L1cm = -params.lc1() * Vector3d::UnitZ();
  // Frame L2's origin is located at the elbow outboard frame.
  const Vector3d p_L2L2cm = -params.lc2() * Vector3d::UnitZ();

  // Define each link's spatial inertia about their respective COM.
  RotationalInertia<double> G1_Bcm =
      UnitInertia<double>::StraightLine(params.Ic1(), Vector3d::UnitY());
  SpatialInertia<double> M1_L1o =
      SpatialInertia<double>::MakeFromCentralInertia(
          params.m1(), p_L1L1cm, G1_Bcm);

  UnitInertia<double> G2_Bcm =
      UnitInertia<double>::StraightLine(params.Ic2(), Vector3d::UnitY());
  SpatialInertia<double> M2_L2o =
      SpatialInertia<double>::MakeFromCentralInertia(
          params.m2(), p_L2L2cm, G2_Bcm);

  // Add a rigid body to model each link.
  const RigidBody<double>& link1 = plant->AddRigidBody("Link1", M1_L1o);
  const RigidBody<double>& link2 = plant->AddRigidBody("Link2", M2_L2o);

  if (geometry_system != nullptr) {
    // A rod geometry for link 1.
    const double rod1_radius = 0.05;
    plant->RegisterGeometry(
        link1,
        /* Pose X_L1G of the geometry frame G in the body frame L1 of link 1. */
        Isometry3d{Translation3d(-params.l1() / 2.0 * Vector3d::UnitZ())},
        Cylinder(rod1_radius, params.l1()), geometry_system);

    // A rod geometry for link 2.
    const double rod2_radius = 0.05;
    plant->RegisterGeometry(
        link2,
        Isometry3d{Translation3d(-params.l2() / 2.0 * Vector3d::UnitZ())},
        Cylinder(rod2_radius, params.l2()), geometry_system);
  }

  plant->AddJoint<RevoluteJoint>(
      "ShoulderJoint",
      /* Shoulder inboard frame Si IS the the world frame W. */
      plant->get_world_body(), {},
      /* Shoulder outboard frame So IS frame L1. */
      link1, {},
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  plant->AddJoint<RevoluteJoint>(
      "ElbowJoint",
      link1,
      /* Pose of the elbow inboard frame Ei in Link 1's frame. */
      Isometry3d(Translation3d(-params.l1() * Vector3d::UnitZ())),
      link2,
      /* Elbow outboard frame Eo IS frame L2 for link 2. */
      {},
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -params.g() * Vector3d::UnitZ());

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

#if 0
template<typename T>
MbpAcrobotPlant<T>::MbpAcrobotPlant(
    double m1, double m2, double l1, double l2,
    double lc1, double lc2, double Ic1, double Ic2,
    double b1, double b2, double g) :
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
  DiagramBuilder<T> builder;
  model_ = builder.template AddSystem(MakeMultibodyPlant());
  builder.BuildInto(this);

  // Some very basic verification that the model is what we expect it to be.
  DRAKE_DEMAND(model_->num_positions() == 2);
  DRAKE_DEMAND(model_->num_velocities() == 2);
  DRAKE_DEMAND(model_->num_states() == 4);
}

template<typename T>
MbpAcrobotPlant<T>::MbpAcrobotPlant(
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);
  DiagramBuilder<T> builder;
  model_ = builder.template AddSystem(MakeMultibodyPlant());
  RegisterGeometry(geometry_system);

  // Export input and output ports here.
  // Export geometry id and poses ports.

  builder.BuildInto(this);
}

#if 0
template<typename T>
template<typename U>
MbpAcrobotPlant<T>::MbpAcrobotPlant(
    const MbpAcrobotPlant<U> &other) :
    MbpAcrobotPlant(other.m1(), other.m2(),
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
#endif

#if 0
template <typename T>
void MbpAcrobotPlant<T>::CopyStateOut(const systems::Context<T>& context,
                                   systems::BasicVector<T>* output) const {
  output->set_theta1(shoulder_->get_angle(context));
  output->set_theta1dot(shoulder_->get_angular_rate(context));
  output->set_theta2(elbow_->get_angle(context));
  output->set_theta2dot(elbow_->get_angular_rate(context));
}
#endif

#if 0
template <typename T>
const OutputPort<T>& MbpAcrobotPlant<T>::get_geometry_id_output_port()
const {
  return systems::System<T>::get_output_port(geometry_id_port_);
}

template <typename T>
const OutputPort<T>& MbpAcrobotPlant<T>::get_geometry_pose_output_port()
const {
  return systems::System<T>::get_output_port(geometry_pose_port_);
}

template <typename T>
const InputPortDescriptor<T>& MbpAcrobotPlant<T>::get_input_port() const {
  return systems::System<T>::get_input_port(applied_torque_input_);
}

template <typename T>
const OutputPort<T>& MbpAcrobotPlant<T>::get_state_output_port() const {
  return systems::System<T>::get_output_port(state_output_port_);
}
#endif

template<typename T>
std::unique_ptr<MultibodyPlant<T>>
MakeMultibodyPlant() const {
  auto model = std::make_unique<MultibodyPlant<T>>();
  // ... add stuff...
  return model;
}

#if 0
template<typename T>
void MbpAcrobotPlant<T>::BuildMultibodyTreeModel() {
  DiagramBuilder<double> builder;
  model_ = builder.template AddSystem<MultibodyPlant>();
  DRAKE_DEMAND(model_ != nullptr);
  //builder.ExportInput(model_->get_input_port());
  //builder.ExportOutput(model_->get_output_port());
  builder.BuildInto(this);

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
#endif


template<typename T>
void MbpAcrobotPlant<T>::RegisterGeometry(
    geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  // Here use MBP's API to declare geometry on a GS
#if 0
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
#endif
}
#endif
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake
