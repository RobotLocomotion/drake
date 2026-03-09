#include "drake/examples/hydroelastic/ball_plate/make_ball_plate_plant.h"

#include <string>
#include <utility>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace ball_plate {

using Eigen::Vector3d;
using geometry::AddCompliantHydroelasticProperties;
using geometry::AddContactMaterial;
using geometry::ProximityProperties;
using geometry::Sphere;
using math::RigidTransformd;
using math::RotationMatrixd;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialInertia;

namespace {

// Add tiny visual cylinders on the ±x,y,z axes of the ball to appreciate
// its rotation.
void AddTinyVisualCylinders(const RigidBody<double>& ball, double radius,
                            MultibodyPlant<double>* plant) {
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
  const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
  const double visual_radius = 0.05 * radius;
  const geometry::Cylinder spot(visual_radius, visual_radius);
  // N.B. We do not place the cylinder's cap exactly on the sphere surface to
  // avoid visualization artifacts when the surfaces are kissing.
  const double radial_offset = radius - 0.45 * visual_radius;
  // Let S be the sphere's frame (at its center) and C be the cylinder's
  // frame (at its center). The goal is to get Cz (frame C's z axis)
  // aligned with p_SC, with Cx and Cy arbitrary.
  // @return X_SC the pose of the spot cylinder given p_SC.
  auto spot_pose = [](const Vector3<double>& p_SC) {
    return RigidTransformd(RotationMatrixd::MakeFromOneVector(p_SC, 2 /*z*/),
                           p_SC);
  };
  plant->RegisterVisualGeometry(ball, spot_pose({radial_offset, 0., 0.}), spot,
                                "sphere_x+", red);
  plant->RegisterVisualGeometry(ball, spot_pose({-radial_offset, 0., 0.}), spot,
                                "sphere_x-", red);
  plant->RegisterVisualGeometry(ball, spot_pose({0., radial_offset, 0.}), spot,
                                "sphere_y+", green);
  plant->RegisterVisualGeometry(ball, spot_pose({0., -radial_offset, 0.}), spot,
                                "sphere_y-", green);
  plant->RegisterVisualGeometry(ball, spot_pose({0., 0., radial_offset}), spot,
                                "sphere_z+", blue);
  plant->RegisterVisualGeometry(ball, spot_pose({0., 0., -radial_offset}), spot,
                                "sphere_z-", blue);
}

}  // namespace

void AddBallPlateBodies(double radius, double mass, double hydroelastic_modulus,
                        double dissipation,
                        const CoulombFriction<double>& surface_friction,
                        double resolution_hint_factor,
                        MultibodyPlant<double>* plant) {
  DRAKE_DEMAND(plant != nullptr);

  // Add the ball. Let B be the ball's frame (at its center). The ball's
  // center of mass Bcm is coincident with Bo.
  const RigidBody<double>& ball = plant->AddRigidBody(
      "Ball", SpatialInertia<double>::SolidSphereWithMass(mass, radius));
  // Set up mechanical properties of the ball.
  ProximityProperties ball_props;
  AddContactMaterial(dissipation, {} /* point stiffness */, surface_friction,
                     &ball_props);
  AddCompliantHydroelasticProperties(radius * resolution_hint_factor,
                                     hydroelastic_modulus, &ball_props);
  plant->RegisterCollisionGeometry(ball, RigidTransformd::Identity(),
                                   Sphere(radius), "collision",
                                   std::move(ball_props));
  const Vector4<double> orange(1.0, 0.55, 0.0, 0.2);
  plant->RegisterVisualGeometry(ball, RigidTransformd::Identity(),
                                Sphere(radius), "visual", orange);
  AddTinyVisualCylinders(ball, radius, plant);

  // Add the dinner plate.
  drake::multibody::Parser parser(plant);
  parser.AddModelsFromUrl("package://drake_models/dishes/plate_8in.sdf");

  // Add the floor. Assume the frame named "Floor" is in the SDFormat file.
  parser.AddModelsFromUrl(
      "package://drake/examples/hydroelastic/ball_plate/floor.sdf");
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("Floor"),
                    RigidTransformd::Identity());

  // Gravity acting in the -z direction.
  plant->mutable_gravity_field().set_gravity_vector(Vector3d{0, 0, -9.81});
}

}  // namespace ball_plate
}  // namespace examples
}  // namespace drake
