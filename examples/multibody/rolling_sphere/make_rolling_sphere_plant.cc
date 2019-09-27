#include "drake/examples/multibody/rolling_sphere/make_rolling_sphere_plant.h"

#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using math::RigidTransformd;
using geometry::Sphere;
using geometry::HalfSpace;
using geometry::SceneGraph;

std::unique_ptr<drake::multibody::MultibodyPlant<double>> MakeBouncingBallPlant(
    double radius, double mass, double elastic_modulus, double dissipation,
    const CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W, SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>();

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  if (scene_graph != nullptr) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, 0);

    const RigidTransformd X_WG(HalfSpace::MakePose(normal_W, point_W));
    // A half-space for the ground geometry.
    plant->RegisterCollisionGeometry(plant->world_body(), X_WG, HalfSpace(),
                                     "collision", surface_friction);

    // Add visual for the ground.
    plant->RegisterVisualGeometry(plant->world_body(), X_WG, HalfSpace(),
                                  "visual");

    // Add sphere geometry for the ball.
    // Pose of sphere geometry S in body frame B.
    const RigidTransformd X_BS = RigidTransformd::Identity();
    geometry::GeometryId sphere_geometry = plant->RegisterCollisionGeometry(
        ball, X_BS, Sphere(radius), "collision", surface_friction);

    // Set material properties for hydroelastics.
    plant->set_elastic_modulus(sphere_geometry, elastic_modulus);
    plant->set_hunt_crossley_dissipation(sphere_geometry, dissipation);

    // Add visual for the ball.
    const Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
    plant->RegisterVisualGeometry(ball, X_BS, Sphere(radius), "visual", orange);

    // We add a few purple spots so that we can appreciate the sphere's
    // rotation.
    const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
    const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
    const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
    const double visual_radius = 0.2 * radius;
    const geometry::Cylinder spot(visual_radius, visual_radius);
    // N.B. We do not place the cylinder's cap exactly on the sphere surface to
    // avoid visualization artifacts when the surfaces are kissing.
    const double radial_offset = radius - 0.45 * visual_radius;
    auto spot_pose = [](const Vector3<double>& position) {
      // The cylinder's z-axis is defined as the normalized vector from the
      // sphere's origin to the cylinder's center `position`.
      const Vector3<double> axis = position.normalized();
      return RigidTransformd(
          Eigen::Quaterniond::FromTwoVectors(Vector3<double>::UnitZ(), axis),
          position);
    };
    plant->RegisterVisualGeometry(ball, spot_pose({radial_offset, 0., 0.}),
                                  spot, "sphere_x+", red);
    plant->RegisterVisualGeometry(ball, spot_pose({-radial_offset, 0., 0.}),
                                  spot, "sphere_x-", red);
    plant->RegisterVisualGeometry(ball, spot_pose({0., radial_offset, 0.}),
                                  spot, "sphere_y+", green);
    plant->RegisterVisualGeometry(ball, spot_pose({0., -radial_offset, 0.}),
                                  spot, "sphere_y-", green);
    plant->RegisterVisualGeometry(ball, spot_pose({0., 0., radial_offset}),
                                  spot, "sphere_z+", blue);
    plant->RegisterVisualGeometry(ball, spot_pose({0., 0., -radial_offset}),
                                  spot, "sphere_z-", blue);
  }

  // Gravity acting in the -z direction.
  plant->mutable_gravity_field().set_gravity_vector(gravity_W);

  return plant;
}

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
