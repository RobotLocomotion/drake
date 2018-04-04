#include "drake/examples/multibody/bouncing_ball/make_bouncing_ball_plant.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

using geometry::GeometrySystem;
using geometry::Sphere;
using geometry::HalfSpace;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeBouncingBallPlant(double radius, double mass,
                      const Vector3<double>& gravity_W,
                      geometry::GeometrySystem<double>* geometry_system) {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  if (geometry_system != nullptr) {
    plant->RegisterAsSourceForGeometrySystem(geometry_system);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, 0);

    // A half-space for the ground geometry.
    plant->RegisterCollisionGeometry(
        plant->world_body(),
        HalfSpace::MakePose(normal_W, point_W), HalfSpace(), geometry_system);

    // Add sphere geometry for the ball.
    plant->RegisterCollisionGeometry(
        ball,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        Isometry3<double>::Identity(),
        Sphere(radius), geometry_system);
  }

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(gravity_W);

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
