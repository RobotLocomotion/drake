#include "drake/multibody/benchmarks/pendulum/make_pendulum_plant.h"

#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace pendulum {

using Eigen::Isometry3d;
using Eigen::Translation3d;
using Eigen::Vector3d;

using geometry::Cylinder;
using geometry::FrameId;
using geometry::SceneGraph;
using geometry::Sphere;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::RigidBody;
using drake::multibody::RotationalInertia;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakePendulumPlant(const PendulumParameters& params,
                  SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  // Position of the com of the pendulum's body (in this case a point mass) in
  // the body's frame. The body's frame's origin Bo is defined to be at the
  // world's origin Wo, through which the rotation axis passes.
  // With the pendulum at rest pointing in the downwards -z direction, the body
  // frame B and the world frame W are coincident.
  const Vector3<double> p_BoBcm_B = -params.l() * Vector3<double>::UnitZ();

  // Define the point mass spatial inertia about its body frame origin Bo.
  // We define the body frame B to have its origin Bo coincident with Wo at all
  // times, and it rotates about the a pin joint's y-axis.
  UnitInertia<double> G_Bo =
      UnitInertia<double>::PointMass(p_BoBcm_B);
  SpatialInertia<double> M_Bo(params.m(), p_BoBcm_B, G_Bo);

  // Add a rigid body to model the point mass.
  const RigidBody<double>& point_mass =
      plant->AddRigidBody(params.body_name(), M_Bo);

  if (scene_graph != nullptr) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);
    // Pose of the sphere used to visualize the point mass in the body frame B.
    const Isometry3d X_BGs{
        Translation3d(-params.l() * Vector3d::UnitZ())};
    plant->RegisterVisualGeometry(
        point_mass, X_BGs, Sphere(params.point_mass_radius()), scene_graph);

    // Pose of the cylinder used to visualize the massless rod in frame B.
    const Isometry3d X_BGc{
        Translation3d(-params.l() / 2.0 * Vector3d::UnitZ())};
    plant->RegisterVisualGeometry(
        point_mass, X_BGc, Cylinder(params.massless_rod_radius(), params.l()),
        scene_graph);
  }

  const RevoluteJoint<double>& pin = plant->AddJoint<RevoluteJoint>(
      params.pin_joint_name(),
      /* Shoulder inboard frame IS the world frame W. */
      plant->world_body(), {},
      /* Pin joint outboard frame IS the body frame B. */
      point_mass, {},
      Vector3d::UnitY()); /* Pendulum oscillates in the x-z plane. */

  // Add pendulum's actuator at the pin joint.
  plant->AddJointActuator(params.actuator_name(), pin);

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -params.g() * Vector3d::UnitZ());

  plant->Finalize(scene_graph);

  return plant;
}

}  // namespace pendulum
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
