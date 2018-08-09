#include "drake/examples/multibody/bouncing_ball/make_bouncing_ball_plant.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

using geometry::SceneGraph;
using geometry::Cylinder;
using geometry::Sphere;
using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::VisualMaterial;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using Eigen::Isometry3d;

void AddCylinderWithMultiContact(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const RigidBody<double>& body,
    double radius, double length, const CoulombFriction<double>& friction,
    double contact_radius, int num_contacts) {
  const VisualMaterial orange(Vector4<double>(1.0, 0.55, 0.0, 1.0));
  const VisualMaterial red(Vector4<double>(1.0, 0.0, 0.0, 1.0));
  // Add sphere geometry for the ball.
#if 0
  plant->RegisterCollisionGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3d::Identity(), Cylinder(radius - 1.05 * contact_radius, length),
      friction, scene_graph);
#endif

  // Visual for the Cylinder
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3d::Identity(), Cylinder(radius, length), orange, scene_graph);

  // Add a bunch of little spheres to simulate "multi-contact".
  const int nspheres = num_contacts;
  const double contact_spheres_radius = contact_radius;
  for (int i = 0; i < nspheres; ++i) {
    const double theta = 2.0 * i / nspheres * M_PI;
    const double x = cos(theta) * radius;
    const double y = sin(theta) * radius;
    Isometry3<double> X_BG = Isometry3<double>::Identity();
    // Top spheres:
    X_BG.translation() << x, y, length / 2;
    plant->RegisterCollisionGeometry(
        body,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        X_BG,
        Sphere(contact_spheres_radius), friction, scene_graph);
    plant->RegisterVisualGeometry(
        body,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        X_BG,
        Sphere(contact_spheres_radius), red, scene_graph);

    // Bottom spheres:
    X_BG.translation() << x, y, -length / 2;
    plant->RegisterCollisionGeometry(
        body,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        X_BG,
        Sphere(contact_spheres_radius), friction, scene_graph);
    plant->RegisterVisualGeometry(
        body,
        /* Pose X_BG of the geometry frame G in the ball frame B. */
        X_BG,
        Sphere(contact_spheres_radius), red, scene_graph);
  }
}

void AddSphereWithSpokes(
    MultibodyPlant<double>* plant, SceneGraph<double>* scene_graph,
    const RigidBody<double>& body,
    double radius, const CoulombFriction<double>& friction) {

  // Add sphere geometry for the ball.
  const double spoke_radius = radius / 10;
  plant->RegisterCollisionGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3d::Identity(),
      Sphere(radius), friction, scene_graph);

  // Adds visual
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3d::Identity(),
      Sphere(radius), scene_graph);

  // Adds little spherical spokes highlight the sphere's rotation.
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, 0, radius)), Sphere(spoke_radius),
      scene_graph);
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, 0, -radius)),
      Sphere(spoke_radius), scene_graph);
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(radius, 0, 0)), Sphere(spoke_radius),
      scene_graph);
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(-radius, 0, 0)),
      Sphere(spoke_radius), scene_graph);
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, radius, 0)), Sphere(spoke_radius),
      scene_graph);
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the ball frame B. */
      Isometry3<double>(Translation3<double>(0, -radius, 0)),
      Sphere(spoke_radius), scene_graph);

}

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeBouncingBallPlant(int nspheres, double radius, double mass,
                      const CoulombFriction<double>& surface_friction,
                      const Vector3<double>& gravity_W,
                      double dt,
                      geometry::SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>(dt);

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  if (scene_graph != nullptr) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);

    Vector3<double> normal_W(0, 0, 1);
    Vector3<double> point_W(0, 0, 0);

    // A half-space for the ground geometry.
    plant->RegisterCollisionGeometry(
        plant->world_body(),
        HalfSpace::MakePose(normal_W, point_W), HalfSpace(), surface_friction,
        scene_graph);
    plant->RegisterVisualGeometry(
        plant->world_body(),
        HalfSpace::MakePose(normal_W, point_W), HalfSpace(),
        scene_graph);

    // Add visual for the ground.
    plant->RegisterVisualGeometry(
        plant->world_body(), HalfSpace::MakePose(normal_W, point_W),
        HalfSpace(), scene_graph);

    // Add sphere geometry for the ball.
    //AddSphereWithSpokes(plant.get(), scene_graph,
      //                  ball, radius, surface_friction);

    AddCylinderWithMultiContact(
        plant.get(), scene_graph,
        ball, radius, 4 * radius, surface_friction, radius / 20.0, 10);

#if 0
    // Add a bunch of little spheres to simulate "multi-contact".
    const int nspheres = 13;
    const double contact_spheres_radius = radius / 50.0;
    for (int i = 0; i < nspheres; ++i) {
      const double theta = 2.0 * i / nspheres * M_PI;
      const double x = cos(theta) * (radius + 1.01 * contact_spheres_radius);
      const double y = sin(theta) * (radius + 1.01 * contact_spheres_radius);
      Isometry3<double> X_BG = Isometry3<double>::Identity();
      // Top spheres:
      X_BG.translation() << x, y, length / 2;
      plant->RegisterCollisionGeometry(
          ball,
          /* Pose X_BG of the geometry frame G in the ball frame B. */
          X_BG,
          Sphere(contact_spheres_radius), surface_friction, scene_graph);

      // Bottom spheres:
      X_BG.translation() << x, y, -length / 2;
      plant->RegisterCollisionGeometry(
          ball,
          /* Pose X_BG of the geometry frame G in the ball frame B. */
          X_BG,
          Sphere(contact_spheres_radius), surface_friction, scene_graph);
    }
#endif
  }

  if (nspheres == 2) {
    // A second geometry
    const RigidBody<double> &ball2 = plant->AddRigidBody("Ball2", M_Bcm);
    AddSphereWithSpokes(plant.get(), scene_graph,
                        ball2, radius, surface_friction);
  }

  // Gravity acting in the -z direction.
  plant->AddForceElement<UniformGravityFieldElement>(gravity_W);

  // We are done creating the plant.
  plant->Finalize(scene_graph);

  return plant;
}

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
