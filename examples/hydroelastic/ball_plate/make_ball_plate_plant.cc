#include "drake/examples/hydroelastic/ball_plate/make_ball_plate_plant.h"

#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace ball_plate {

using drake::geometry::AddContactMaterial;
using drake::geometry::AddRigidHydroelasticProperties;
using drake::geometry::AddCompliantHydroelasticProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::Box;
using drake::multibody::CoulombFriction;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::math::RigidTransformd;
using Eigen::Vector3d;

namespace {

// Add tiny visual cylinders on the ±x,y,z axes of the ball to appreciate
// its rotation.
void AddTinyVisualCylinders(const RigidBody<double>& ball, double radius,
                            MultibodyPlant<double>* plant) {
  // We will add tiny visual cylinders to appreciate the ball's rotation.
  const Vector4<double> red(1.0, 0.0, 0.0, 1.0);
  const Vector4<double> green(0.0, 1.0, 0.0, 1.0);
  const Vector4<double> blue(0.0, 0.0, 1.0, 1.0);
  const double visual_radius = 0.05 * radius;
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

std::unique_ptr<drake::multibody::MultibodyPlant<double>> MakeBallPlatePlant(
    double mbp_dt, double radius, double mass, double hydroelastic_modulus,
    double dissipation, const CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W,
    double resolution_hint_factor, SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>(mbp_dt);
  DRAKE_DEMAND(scene_graph != nullptr);
  plant->RegisterAsSourceForSceneGraph(scene_graph);

  // Add the ball.
  const RigidBody<double>& ball = plant->AddRigidBody(
      "Ball", SpatialInertia<double>{mass, /*p_PScm_E*/ Vector3<double>::Zero(),
                                     UnitInertia<double>::SolidSphere(radius)});
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
  AddTinyVisualCylinders(ball, radius, plant.get());

  // Add the dinner plate.
  drake::multibody::Parser parser(plant.get());
  std::string plate_file_name = FindResourceOrThrow(
      "drake/examples/hydroelastic/ball_plate/plate_8in.sdf");
  parser.AddModelFromFile(plate_file_name);

  // Add the floor. Assume the frame named "Floor" is in the SDFormat file.
  std::string floor_file_name =
      FindResourceOrThrow("drake/examples/hydroelastic/ball_plate/floor.sdf");
  parser.AddModelFromFile(floor_file_name);
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("Floor"),
                    RigidTransformd::Identity());

  // Gravity acting in the -z direction.
  plant->mutable_gravity_field().set_gravity_vector(gravity_W);

  return plant;
}

}  // namespace ball_plate
}  // namespace multibody
}  // namespace examples
}  // namespace drake
