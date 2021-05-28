#include "drake/examples/hydroelastic/ball_plate/make_ball_plate_plant.h"

#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"

namespace drake {
namespace examples {
namespace multibody {
namespace bouncing_ball {

using drake::geometry::AddContactMaterial;
using drake::geometry::AddRigidHydroelasticProperties;
using drake::geometry::AddSoftHydroelasticProperties;
using drake::geometry::AddSoftHydroelasticPropertiesForHalfSpace;
using drake::geometry::ProximityProperties;
using drake::geometry::SceneGraph;
using drake::geometry::Sphere;
using drake::geometry::Box;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialInertia;
using drake::multibody::UnitInertia;
using drake::math::RigidTransformd;

std::unique_ptr<drake::multibody::MultibodyPlant<double>> MakeBallPlatePlant(
    double mbp_dt, double radius, double mass, double elastic_modulus,
    double dissipation, const CoulombFriction<double>& surface_friction,
    const Vector3<double>& gravity_W,
    double resolution_hint_factor, SceneGraph<double>* scene_graph) {
  auto plant = std::make_unique<MultibodyPlant<double>>(mbp_dt);

  UnitInertia<double> G_Bcm = UnitInertia<double>::SolidSphere(radius);
  SpatialInertia<double> M_Bcm(mass, Vector3<double>::Zero(), G_Bcm);

  const RigidBody<double>& ball = plant->AddRigidBody("Ball", M_Bcm);

  drake::multibody::Parser parser(plant.get());

  if (scene_graph != nullptr) {
    plant->RegisterAsSourceForSceneGraph(scene_graph);

    std::string full_name = FindResourceOrThrow(
        "drake/examples/hydroelastic/ball_plate/"
        "plate_8in.sdf");
    const auto plate_instance_id = parser.AddModelFromFile(full_name);

    // Add sphere geometry for the ball.
    // Pose of sphere geometry S in body frame B.
    const RigidTransformd X_BS = RigidTransformd::Identity();
    // Set material properties for hydroelastics.
    ProximityProperties ball_props;
    AddContactMaterial(elastic_modulus, dissipation, surface_friction,
                       &ball_props);
    AddSoftHydroelasticProperties(radius * resolution_hint_factor, &ball_props);
    plant->RegisterCollisionGeometry(ball, X_BS, Sphere(radius), "collision",
                                     std::move(ball_props));

    // Add visual for the ball.
    const Vector4<double> orange(1.0, 0.55, 0.0, 0.2);
    plant->RegisterVisualGeometry(ball, X_BS, Sphere(radius), "visual", orange);

    // We add a few purple spots so that we can appreciate the sphere's
    // rotation.
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

    // Add a floor, to support the plate
    const double Lx = 0.15, Ly = 0.15, Lz = 2e-3;
    UnitInertia<double> G_Fcm = UnitInertia<double>::SolidBox(Lx, Ly, Lz);
    SpatialInertia<double> M_Fcm(mass, Vector3<double>::Zero(), G_Fcm);
    const RigidBody<double>& floor =
        plant->AddRigidBody("Floor", plate_instance_id, M_Fcm);

    const RigidTransformd X_BF = RigidTransformd::Identity();
    ProximityProperties floor_props;
    const CoulombFriction<double> floor_friction(0.3 /* static friction */,
                                                 0.3 /* dynamic friction */);
    AddContactMaterial(5.0e4 /* elastic modulus */, 5.0 /* dissipation */,
                       floor_friction, &floor_props);
    AddSoftHydroelasticProperties(radius * resolution_hint_factor,
                                  &floor_props);
    plant->RegisterCollisionGeometry(floor, X_BF, Box(Lx, Ly, Lz), "collision",
                                     std::move(floor_props));

    // Add visual for the floor.
    plant->RegisterVisualGeometry(floor, X_BF, Box(Lx, Ly, Lz), "visual",
                                  orange);

    const drake::multibody::Frame<double>& floor_base_frame =
        plant->GetFrameByName("Floor");
    plant->WeldFrames(plant->world_frame(), floor_base_frame,
                      RigidTransformd());
  }

  // Gravity acting in the -z direction.
  plant->mutable_gravity_field().set_gravity_vector(gravity_W);

  return plant;
}

}  // namespace bouncing_ball
}  // namespace multibody
}  // namespace examples
}  // namespace drake
