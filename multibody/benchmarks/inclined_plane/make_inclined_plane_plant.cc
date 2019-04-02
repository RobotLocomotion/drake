#include "drake/multibody/benchmarks/inclined_plane/make_inclined_plane_plant.h"

#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace multibody {
namespace benchmarks {
namespace inclined_plane {

using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::Sphere;
using math::RigidTransformd;
using Eigen::AngleAxisd;
using Eigen::Matrix3d;
using Eigen::Vector3d;

void AddInclinedPlaneToPlant(
    double radius, double mass, double slope,
    const CoulombFriction<double>& surface_friction, double gravity,
    MultibodyPlant<double>* plant) {
  DRAKE_THROW_UNLESS(plant != nullptr);

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3d::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  // Orientation of a plane frame P with its z axis normal to the plane and its
  // x axis pointing down the plane.
  const Matrix3d R_WP(AngleAxisd(slope, Vector3d::UnitY()));

  const Vector3d normal_W = R_WP.col(2);
  // Offset the plane so that with the rolling object's center at x = (0, 0, 0)
  // (the default initial condition) there is a contact point at point_W.
  const Vector3d point_W = -normal_W * radius;

  // A half-space for the inclined plane geometry.
  const RigidTransformd X_WG(HalfSpace::MakePose(normal_W, point_W));
  plant->RegisterCollisionGeometry(plant->world_body(), X_WG, HalfSpace(),
                                   "collision", surface_friction);

  // Visual for the ground.
  plant->RegisterVisualGeometry(plant->world_body(), X_WG, HalfSpace(),
                                "visual");

  // Add sphere geometry for the ball.
  // Pose X_BG of geometry frame G in the ball frame B is an identity transform.
  const RigidTransformd X_BG;  // Identity transform.
  plant->RegisterCollisionGeometry(ball, X_BG, Sphere(radius), "collision",
                                   surface_friction);

  // Visual for the ball.
  const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  plant->RegisterVisualGeometry(ball, X_BG, Sphere(radius), "visual1", orange);

  // Adds little spherical spokes to highlight the sphere's rotation.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  plant->RegisterVisualGeometry(ball,
                                // Pose of 1st spoke frame in the ball frame B.
                                RigidTransformd(Vector3d(0, 0, radius)),
                                Sphere(radius / 5), "visual2", red);
  plant->RegisterVisualGeometry(ball,
                                // Pose of 2nd spoke frame in the ball frame B.
                                RigidTransformd(Vector3d(0, 0, -radius)),
                                Sphere(radius / 5), "visual3", red);
  plant->RegisterVisualGeometry(ball,
                                // Pose of 3rd spoke frame in the ball frame B.
                                RigidTransformd(Vector3d(radius, 0, 0)),
                                Sphere(radius / 5), "visual4", red);
  plant->RegisterVisualGeometry(ball,
                                // Pose of 4th spoke frame in the ball frame B.
                                RigidTransformd(Vector3d(-radius, 0, 0)),
                                Sphere(radius / 5), "visual5", red);

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(-gravity *
                                                     Vector3d::UnitZ());
}

}  // namespace inclined_plane
}  // namespace benchmarks
}  // namespace multibody
}  // namespace drake
