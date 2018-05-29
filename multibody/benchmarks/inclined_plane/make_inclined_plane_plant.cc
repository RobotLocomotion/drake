#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::Sphere;
using multibody_plant::CoulombFriction;
using multibody::multibody_plant::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::UniformGravityFieldElement;
using multibody::UnitInertia;
using Eigen::AngleAxisd;

std::unique_ptr<MultibodyPlant<double>> MakeInclinedPlanePlant(
    double radius, double mass, double slope,
    const CoulombFriction<double>& surface_friction, double gravity,
    SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);

  auto plant = std::make_unique<MultibodyPlant<double>>();

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  plant->RegisterAsSourceForSceneGraph(scene_graph);

  // Orientation of a plane frame P with its z axis normal to the plane and its
  // x axis pointing down the plane.
  const Matrix3<double> R_WP(AngleAxisd(slope, Vector3<double>::UnitY()));

  const Vector3<double> normal_W = R_WP.col(2);
  // Offset the plane so that with the rolling object's center at x = (0, 0, 0)
  // (the default initial condition) there is a contact point at point_W.
  const Vector3<double> point_W = -normal_W * radius;

  // A half-space for the inclined plane geometry.
  plant->RegisterCollisionGeometry(plant->world_body(),
                                   HalfSpace::MakePose(normal_W, point_W),
                                   HalfSpace(), surface_friction, scene_graph);

  // Add sphere geometry for the ball.
  plant->RegisterCollisionGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>::Identity(), Sphere(radius), surface_friction,
      scene_graph);

  // Adds little spherical spokes highlight the sphere's rotation.
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, 0, radius)), Sphere(radius / 5),
      scene_graph);
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, 0, -radius)),
      Sphere(radius / 5), scene_graph);
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(radius, 0, 0)), Sphere(radius / 5),
      scene_graph);
  plant->RegisterVisualGeometry(
      ball,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(-radius, 0, 0)),
      Sphere(radius / 5), scene_graph);

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(
      -gravity * Vector3<double>::UnitZ());

  // We are done creating the plant.
  plant->Finalize(scene_graph);

  return plant;
}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
