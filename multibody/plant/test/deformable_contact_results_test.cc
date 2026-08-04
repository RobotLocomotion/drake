#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/geometry/proximity_properties.h"
#include "drake/multibody/plant/compliant_contact_manager.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::GeometryInstance;
using drake::geometry::ProximityProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::systems::Context;
using Eigen::Vector3d;

namespace drake {
namespace multibody {
namespace internal {

// Tests that querying contact results in the presence of deformable contact
// doesn't throw. See #20187.
GTEST_TEST(CompliantContactManagerTest, ContactResultsWithDeformable) {
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = 1.0e-3;
  plant_config.discrete_contact_approximation = "sap";
  plant_config.use_sampled_output_ports = false;  // We're not stepping time.
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  /* Add a hydro ground weld to the world*/
  ProximityProperties hydro_proximity_properties;
  geometry::AddCompliantHydroelasticProperties(
      /* resolution_hint */ 1.0, 1e6, &hydro_proximity_properties);
  geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                               &hydro_proximity_properties);
  plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                  geometry::Box(10, 10, 10), "ground_collision",
                                  hydro_proximity_properties);
  /* Add a rigid box that has compliant hydro proximity properties and collide
   with the ground. */
  const auto M = SpatialInertia<double>::SolidCubeWithMass(1.0, 0.1);
  const auto& body1 = plant.AddRigidBody("hydro body", M);
  plant.RegisterCollisionGeometry(body1, RigidTransformd(Vector3d(5, 5, 5)),
                                  geometry::Box(1, 1, 1), "hydro box",
                                  hydro_proximity_properties);

  /* Add a rigid box that has point contact proximity properties and collide
   with the ground (but not with the other body1). */
  ProximityProperties point_proximity_properties;
  const auto& body2 = plant.AddRigidBody("point contact body", M);
  geometry::AddContactMaterial(1e6, 1.0, CoulombFriction<double>(1.0, 1.0),
                               &point_proximity_properties);
  plant.RegisterCollisionGeometry(body2, RigidTransformd(Vector3d(-5, -5, 5)),
                                  geometry::Box(1, 1, 1), "point contact box",
                                  point_proximity_properties);
  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();
  /* Add a deformable sphere that collides with the ground but not with any
   other rigid bodies. */
  auto deformable_geometry = std::make_unique<GeometryInstance>(
      RigidTransformd(Vector3d(0, 0, 5)), std::make_unique<Sphere>(1.0),
      "sphere");
  ProximityProperties deformable_proximity_props;
  geometry::AddContactMaterial({}, {}, CoulombFriction<double>(1.0, 1.0),
                               &deformable_proximity_props);
  deformable_geometry->set_proximity_properties(
      std::move(deformable_proximity_props));
  multibody::fem::DeformableBodyConfig<double> body_config;
  constexpr double kRezHint = 10.0;
  deformable_model.RegisterDeformableBody(std::move(deformable_geometry),
                                          body_config, kRezHint);
  plant.Finalize();

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  const auto& plant_context = plant.GetMyContextFromRoot(*context);

  const auto& contact_results =
      plant.get_contact_results_output_port().Eval<ContactResults<double>>(
          plant_context);
  EXPECT_EQ(contact_results.num_point_pair_contacts(), 1);
  EXPECT_EQ(contact_results.num_hydroelastic_contacts(), 1);
  EXPECT_EQ(contact_results.num_deformable_contacts(), 1);
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake
