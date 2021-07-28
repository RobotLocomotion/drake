#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"

#include <optional>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace acrobot {

using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::FrameId;
using geometry::SceneGraph;
using geometry::Sphere;
using drake::math::RigidTransformd;

std::unique_ptr<MultibodyPlant<double>>
MakeAcrobotPlant(const AcrobotParameters& params, bool finalize,
                 SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);

  // COM's positions in each link (L1/L2) frame:
  // Frame L1's origin is located at the shoulder outboard frame.
  const Vector3d p_L1L1cm = -params.lc1() * Vector3d::UnitZ();
  // Frame L2's origin is located at the elbow outboard frame.
  const Vector3d p_L2L2cm = -params.lc2() * Vector3d::UnitZ();

  // Define each link's spatial inertia about their respective COM.
  UnitInertia<double> G1_Bcm =
      UnitInertia<double>::StraightLine(params.Gc1(), Vector3d::UnitZ());
  SpatialInertia<double> M1_L1o =
      SpatialInertia<double>::MakeFromCentralInertia(
          params.m1(), p_L1L1cm, G1_Bcm * params.m1());

  UnitInertia<double> G2_Bcm =
      UnitInertia<double>::StraightLine(params.Gc2(), Vector3d::UnitZ());
  SpatialInertia<double> M2_L2o =
      SpatialInertia<double>::MakeFromCentralInertia(
          params.m2(), p_L2L2cm, G2_Bcm * params.m2());

  // Add a rigid body to model each link.
  const RigidBody<double>& link1 = plant->AddRigidBody(
      params.link1_name(), M1_L1o);
  const RigidBody<double>& link2 = plant->AddRigidBody(
      params.link2_name(), M2_L2o);

  if (scene_graph != nullptr) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);

    // Pose of the geometry for link 1 in the link's frame.
    const RigidTransformd X_L1G1(-params.l1() / 2.0 * Vector3d::UnitZ());
    plant->RegisterVisualGeometry(link1, X_L1G1,
                                  Cylinder(params.r1(), params.l1()), "visual");

    // Pose of the geometry for link 2 in the link's frame.
    const RigidTransformd X_L2G2(-params.l2() / 2.0 * Vector3d::UnitZ());
    plant->RegisterVisualGeometry(link2, X_L2G2,
                                  Cylinder(params.r2(), params.l2()), "visual");

    // Register some (anchored) geometry to the world.
    const RigidTransformd X_WG;  // Default is identity transform.
    plant->RegisterVisualGeometry(
        plant->world_body(), X_WG,
        Sphere(params.l1() / 8.0), /* Arbitrary radius to decorate the model. */
        "visual");
  }

  plant->AddJoint<RevoluteJoint>(
      params.shoulder_joint_name(),
      /* Shoulder inboard frame Si IS the world frame W. */
      plant->world_body(), std::nullopt,
      /* Shoulder outboard frame So IS frame L1. */
      link1, std::nullopt,
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  // Pose of the elbow inboard frame Ei in Link 1's frame.
  const RigidTransformd X_link1_Ei(-params.l1() * Vector3d::UnitZ());
  const RevoluteJoint<double>& elbow = plant->AddJoint<RevoluteJoint>(
      params.elbow_joint_name(),
      link1,
      X_link1_Ei,
      link2,
      /* Elbow outboard frame Eo IS frame L2 for link 2. */
      std::optional<RigidTransformd>{},  // `nullopt` is ambiguous
      Vector3d::UnitY()); /* acrobot oscillates in the x-z plane. */

  // Add acrobot's actuator at the elbow joint.
  plant->AddJointActuator(params.actuator_name(), elbow);

  // Gravity acting in the -z direction.
  plant->mutable_gravity_field().set_gravity_vector(
      -params.g() * Vector3d::UnitZ());

  // We are done creating the plant.
  if (finalize) plant->Finalize();

  return plant;
}

}  // namespace acrobot
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
