#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::FrameId;
using geometry::GeometrySystem;
using geometry::Sphere;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeAcrobotPlant(
    const AcrobotParameters& params,
    geometry::GeometrySystem<double>* geometry_system) {
  auto plant = std::make_unique<MultibodyPlant<double>>();
  //plant->set_name("Acrobot");

  // COM's positions in each link (L1/L2) frame:
  // Frame L1's origin is located at the shoulder outboard frame.
  const Vector3d p_L1L1cm = -params.lc1() * Vector3d::UnitZ();
  // Frame L2's origin is located at the elbow outboard frame.
  const Vector3d p_L2L2cm = -params.lc2() * Vector3d::UnitZ();

  // Define each link's spatial inertia about their respective COM.
  UnitInertia<double> G1_Bcm =
      UnitInertia<double>::StraightLine(params.Ic1(), Vector3d::UnitZ());
  SpatialInertia<double> M1_L1o =
      SpatialInertia<double>::MakeFromCentralInertia(
          params.m1(), p_L1L1cm, G1_Bcm);

  UnitInertia<double> G2_Bcm =
      UnitInertia<double>::StraightLine(params.Ic2(), Vector3d::UnitZ());
  SpatialInertia<double> M2_L2o =
      SpatialInertia<double>::MakeFromCentralInertia(
          params.m2(), p_L2L2cm, G2_Bcm);

  // Add a rigid body to model each link.
  const RigidBody<double>& link1 = plant->AddRigidBody(
      params.link1_name(), M1_L1o);
  const RigidBody<double>& link2 = plant->AddRigidBody(
      params.link2_name(), M2_L2o);

  if (geometry_system != nullptr) {
    // Pose of the geometry for link 1 in the link's frame.
    const Isometry3d X_L1G1{
        Translation3d(-params.l1() / 2.0 * Vector3d::UnitZ())};
    plant->RegisterVisualGeometry(
        link1, X_L1G1, Cylinder(params.r1(), params.l1()), geometry_system);

    // Pose of the geometry for link 2 in the link's frame.
    const Isometry3d X_L2G2{
        Translation3d(-params.l2() / 2.0 * Vector3d::UnitZ())};
    plant->RegisterVisualGeometry(
        link2, X_L2G2, Cylinder(params.r2(), params.l2()), geometry_system);

    // Register some anchored geometry to the world.
    plant->RegisterAnchoredVisualGeometry(
        Isometry3d::Identity(), /* X_WG */
        Sphere(params.l1() / 8.0), geometry_system);
  }

  plant->AddJoint<RevoluteJoint>(
      params.shoulder_joint_name(),
      /* Shoulder inboard frame Si IS the the world frame W. */
      plant->get_world_body(), {},
      /* Shoulder outboard frame So IS frame L1. */
      link1, {},
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  const RevoluteJoint<double>& elbow = plant->AddJoint<RevoluteJoint>(
      params.elbow_joint_name(),
      link1,
      /* Pose of the elbow inboard frame Ei in Link 1's frame. */
      Isometry3d(Translation3d(-params.l1() * Vector3d::UnitZ())),
      link2,
      /* Elbow outboard frame Eo IS frame L2 for link 2. */
      {},
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  // Add acrobot's actuator at the elbow joint.
  plant->AddJointActuator(params.actuator_name(), elbow);

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -params.g() * Vector3d::UnitZ());

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
