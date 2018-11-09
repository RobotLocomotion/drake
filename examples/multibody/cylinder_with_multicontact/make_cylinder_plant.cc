#include "drake/examples/multibody/cylinder_with_multicontact/make_cylinder_plant.h"

#include "drake/multibody/multibody_tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace cylinder_with_multicontact {

using geometry::Cylinder;
using geometry::HalfSpace;
using geometry::SceneGraph;
using geometry::Sphere;
using geometry::VisualMaterial;
using drake::multibody::multibody_plant::CoulombFriction;
using drake::multibody::multibody_plant::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UniformGravityFieldElement;
using drake::multibody::UnitInertia;
using Eigen::Isometry3d;

void AddCylinderWithMultiContact(
    MultibodyPlant<double>* plant, const RigidBody<double>& body,
    double radius, double length, const CoulombFriction<double>& friction,
    double contact_spheres_radius, int num_contacts) {
  const VisualMaterial orange(Vector4<double>(1.0, 0.55, 0.0, 1.0));
  const VisualMaterial red(Vector4<double>(1.0, 0.0, 0.0, 1.0));

  // Visual for the Cylinder
  plant->RegisterVisualGeometry(
      body,
      /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
      Isometry3d::Identity(), Cylinder(radius, length), "visual", orange);

  // Add a bunch of little spheres to simulate "multi-contact".
  for (int i = 0; i < num_contacts; ++i) {
    const double theta = 2.0 * i / num_contacts * M_PI;
    const double x = cos(theta) * radius;
    const double y = sin(theta) * radius;
    Isometry3<double> X_BG = Isometry3<double>::Identity();
    // Top spheres:
    /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
    X_BG.translation() << x, y, length / 2;
    plant->RegisterCollisionGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "collision_top_" + std::to_string(i),
        friction);
    plant->RegisterVisualGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "visual_top_" + std::to_string(i), red);

    // Bottom spheres:
    X_BG.translation() << x, y, -length / 2;
    plant->RegisterCollisionGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "collision_bottom_" + std::to_string(i),
        friction);
    plant->RegisterVisualGeometry(
        body,
        X_BG,
        Sphere(contact_spheres_radius), "visual_bottom_" + std::to_string(i),
        red);
  }
}

std::unique_ptr<drake::multibody::multibody_plant::MultibodyPlant<double>>
MakeCylinderPlant(double radius, double length, double mass,
                  const CoulombFriction<double>& surface_friction,
                  const Vector3<double>& gravity_W, double dt,
                  geometry::SceneGraph<double>* scene_graph) {
  DRAKE_DEMAND(scene_graph != nullptr);

  auto plant = std::make_unique<MultibodyPlant<double>>(dt);
  plant->RegisterAsSourceForSceneGraph(scene_graph);

  UnitInertia<double> G_Bcm =
      UnitInertia<double>::SolidCylinder(radius, length);

  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);
  const RigidBody<double>& cylinder = plant->AddRigidBody("Cylinder", M_Bcm);

  // The radius of the small spheres used to emulate multicontact.
  const double contact_radius = radius / 20.0;

  // The number of small spheres places at each rim of the cylinder to emulate
  // multicontact.
  const int num_contact_spheres = 10;

  // Add geometry to the cylinder for both contact and visualization.
  AddCylinderWithMultiContact(
      plant.get(),
      cylinder, radius, length, surface_friction,
      contact_radius, num_contact_spheres);

  // Add a model for the ground.
  Vector3<double> normal_W(0, 0, 1);
  Vector3<double> point_W(0, 0, 0);

  // A half-space for the ground geometry.
  plant->RegisterCollisionGeometry(
      plant->world_body(),
      HalfSpace::MakePose(normal_W, point_W), HalfSpace(), "collision",
      surface_friction);

  // Add visual for the ground.
  plant->RegisterVisualGeometry(
      plant->world_body(), HalfSpace::MakePose(normal_W, point_W),
      HalfSpace(), "visual");

  plant->AddForceElement<UniformGravityFieldElement>(gravity_W);

  // We are done creating the plant.
  plant->Finalize();

  return plant;
}

}  // namespace cylinder_with_multicontact
}  // namespace multibody
}  // namespace examples
}  // namespace drake
