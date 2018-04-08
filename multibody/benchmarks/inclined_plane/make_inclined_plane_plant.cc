#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

using geometry::GeometrySystem;
using geometry::Sphere;
using geometry::HalfSpace;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using Eigen::AngleAxisd;

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeInclinedPlanePlant(double radius, double mass, double slope,
                       const CoulombFriction& surface_friction,
                       double gravity,
                       geometry::GeometrySystem<double>* geometry_system) {
  DRAKE_DEMAND(geometry_system != nullptr);

  auto plant = std::make_unique<MultibodyPlant<double>>();

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  plant->RegisterAsSourceForGeometrySystem(geometry_system);

  // Orientation of a plane frame P with its z axis normal to the plane and its
  // x axis pointing down the plane.
  const Matrix3<double> R_WP(AngleAxisd(slope, Vector3<double>::UnitY()));

  const Vector3<double> normal_W = R_WP.col(2);
  // Offset the plane so that with the rolling object's center at x = (0, 0, 0)
  // (the default initial condition) there is a contact point at point_W.
  const Vector3<double> point_W = -normal_W * radius;

  // A half-space for the inclined plane geometry.
  plant->RegisterCollisionGeometry(
      plant->world_body(), HalfSpace::MakePose(normal_W, point_W), HalfSpace(),
      surface_friction, geometry_system);

  // Add sphere geometry for the ball.
  plant->RegisterCollisionGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>::Identity(), Sphere(radius), surface_friction,
      geometry_system);

  // Adds little spherical spokes highlight the sphere's rotation.
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, 0, radius)), Sphere(radius / 5),
      geometry_system);
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, 0, -radius)), Sphere(radius / 5),
      geometry_system);
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(radius, 0, 0)), Sphere(radius / 5),
      geometry_system);
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(-radius, 0, 0)), Sphere(radius / 5),
      geometry_system);

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -gravity * Vector3<double>::UnitZ());

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
