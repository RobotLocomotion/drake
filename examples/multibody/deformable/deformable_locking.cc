#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 5.0, "Desired duration of the simulation [s].");
DEFINE_double(realtime_rate, 1.0, "Desired real time rate.");
DEFINE_double(time_step, 1e-2,
              "Discrete time step for the system [s]. Must be positive.");
DEFINE_double(E, 3e4, "Young's modulus of the deformable bodies [Pa].");
DEFINE_double(nu, 0.4, "Poisson's ratio of the deformable bodies, unitless.");
DEFINE_double(density, 1e3, "Mass density of the deformable bodies [kg/m³].");
DEFINE_double(beta, 0.01,
              "Stiffness damping coefficient for the deformable bodies [1/s].");
DEFINE_string(contact_approximation, "lagged",
              "Type of convex contact approximation. See "
              "multibody::DiscreteContactApproximation for details. Options "
              "are: 'sap', 'lagged', and 'similar'.");
DEFINE_double(
    contact_damping, 10.0,
    "Hunt and Crossley damping for the deformable body, only used when "
    "'contact_approximation' is set to 'lagged' or 'similar' [s/m].");

using drake::geometry::AddContactMaterial;
using drake::geometry::Box;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Mesh;
using drake::geometry::ProximityProperties;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::CoulombFriction;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::fem::DeformableBodyConfig;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace drake {
namespace examples {
namespace {

int do_main() {
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_time_step;
  plant_config.discrete_contact_approximation = FLAGS_contact_approximation;

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  /* Minimum required proximity properties for rigid bodies to interact with
   deformable bodies.
   1. A valid Coulomb friction coefficient, and
   2. A resolution hint. (Rigid bodies need to be tessellated so that collision
   queries can be performed against deformable geometries.) The value dictates
   how fine the mesh used to represent the rigid collision geometry is. */
  ProximityProperties rigid_proximity_props;
  /* Set the friction coefficient close to that of rubber against rubber. */
  const CoulombFriction<double> surface_friction(1.15, 1.15);
  AddContactMaterial({}, {}, surface_friction, &rigid_proximity_props);
  rigid_proximity_props.AddProperty(geometry::internal::kHydroGroup,
                                    geometry::internal::kRezHint, 0.01);
  /* Set up a ground. */
  Box ground{4, 4, 4};
  const RigidTransformd X_WG(Eigen::Vector3d{0, 0, -2});
  plant.RegisterCollisionGeometry(plant.world_body(), X_WG, ground,
                                  "ground_collision", rigid_proximity_props);
  IllustrationProperties illustration_props;
  illustration_props.AddProperty("phong", "diffuse",
                                 Vector4d(0.7, 0.5, 0.4, 0.8));
  plant.RegisterVisualGeometry(plant.world_body(), X_WG, ground,
                               "ground_visual", std::move(illustration_props));

  /* Set up a deformable bodies. */
  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(FLAGS_E);
  deformable_config.set_poissons_ratio(FLAGS_nu);
  deformable_config.set_mass_density(FLAGS_density);
  deformable_config.set_stiffness_damping_coefficient(FLAGS_beta);

  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();

  const std::string torus_vtk = FindResourceOrThrow(
      "drake/examples/multibody/deformable/models/torus.vtk");
  /* Minor diameter of the torus inferred from the vtk file. */
  const double kL = 0.09;

  std::vector<DeformableBodyId> torus_ids;
  for (int i = 0; i < 3; i++) {
    auto torus_mesh = std::make_unique<Mesh>(torus_vtk, 1.0);

    /* Set the initial pose of the i-th torus such that it is slightly above the
    (i-1)th torus. */
    const RigidTransformd X_WB(
        Vector3<double>(0.0, 0.0, kL / 2.0 + 1.25 * kL * i));
    auto torus_instance = std::make_unique<GeometryInstance>(
        X_WB, std::move(torus_mesh), "deformable_torus_" + std::to_string(i));

    /* Minimally required proximity properties for deformable bodies: A valid
    Coulomb friction coefficient. */
    ProximityProperties deformable_proximity_props;
    AddContactMaterial(FLAGS_contact_damping, {}, surface_friction,
                       &deformable_proximity_props);
    torus_instance->set_proximity_properties(deformable_proximity_props);

    /* Registration of all deformable geometries ostensibly requires a
    resolution hint parameter that dictates how the shape is tessellated. In the
    case of a `Mesh` shape, the resolution hint is unused because the shape is
    already tessellated. */
    // TODO(xuchenhan-tri): Though unused, we still asserts the resolution hint
    // is positive. Remove the requirement of a resolution hint for meshed
    // shapes.
    const double unused_resolution_hint = 1.0;
    DeformableBodyId torus_id = deformable_model.RegisterDeformableBody(
        std::move(torus_instance), deformable_config, unused_resolution_hint);

    torus_ids.push_back(torus_id);
  }

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* Add a visualizer that emits LCM messages for visualization. */
  geometry::DrakeVisualizerParams params;
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, nullptr,
                                           params);

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();

  /* Build the simulator and run! */
  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  Context<double>& mutable_root_context = simulator.get_mutable_context();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, &mutable_root_context);

  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);

  /* Initially lock all of the deformables so that they are suspended in the air
  above each other. */
  for (const auto& torus_id : torus_ids) {
    plant.deformable_model().Lock(torus_id, &plant_context);
  }

  /* Advance the simulation time in intervals, unlocking a new deformable body
  at each boundary. */
  const double advance_interval = FLAGS_simulation_time / torus_ids.size();
  for (std::size_t i = 0; i < torus_ids.size(); i++) {
    const auto& torus_id = torus_ids[i];
    plant.deformable_model().Unlock(torus_id, &plant_context);

    simulator.AdvanceTo((i + 1) * advance_interval);
  }

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase locking/unlocking of deformable bodies."
      "Deformable torus bodies are stacked on top of each other and unlocked "
      "one-by-one. Refer to README for instructions on meldis as well as "
      "optional flags.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
