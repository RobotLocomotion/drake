#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>

#include "drake/examples/multibody/deformable/deformable_common.h"
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

using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::fem::DeformableBodyConfig;
using drake::systems::Context;

using drake::examples::deformable::RegisterDeformableTorus;
using drake::examples::deformable::RegisterRigidGround;

namespace drake {
namespace examples {
namespace {

int do_main() {
  systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_time_step;
  plant_config.discrete_contact_approximation = FLAGS_contact_approximation;

  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);
  RegisterRigidGround(&plant);

  /* Set up a deformable bodies. */
  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(FLAGS_E);
  deformable_config.set_poissons_ratio(FLAGS_nu);
  deformable_config.set_mass_density(FLAGS_density);
  deformable_config.set_stiffness_damping_coefficient(FLAGS_beta);

  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();

  /* Add a few deformable torus bodies stacked on top of each other. */
  std::vector<DeformableBodyId> torus_ids;
  for (int i = 0; i < 3; ++i) {
    const double scale = 1.0;
    /* Minor diameter of the torus inferred from the vtk file. */
    const double kL = 0.09 * scale;

    /* Set the initial pose of the i-th torus such that it is slightly above the
     (i-1)th torus. The first torus is set just above the floor surface. */
    const std::string model_name = fmt::format("deformable_torus_{}", i);
    const RigidTransformd X_WB(
        Vector3<double>(0.0, 0.0, kL / 2.0 + 1.25 * kL * i));
    const DeformableBodyId torus_id = RegisterDeformableTorus(
        &deformable_model, model_name, X_WB, deformable_config, scale,
        FLAGS_contact_damping);
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

  /* Initially disable all of the deformable tori so that they are suspended in
   the air above each other. */
  for (const auto& torus_id : torus_ids) {
    plant.deformable_model().Disable(torus_id, &plant_context);
  }

  /* Advance the simulation time in intervals, enabling a new deformable body
   at each boundary. */
  const double advance_interval =
      FLAGS_simulation_time / (torus_ids.size() + 1);
  for (std::size_t i = 0; i < torus_ids.size(); ++i) {
    const auto& torus_id = torus_ids[i];
    plant.deformable_model().Enable(torus_id, &plant_context);

    simulator.AdvanceTo((i + 1) * advance_interval);
  }

  /* Disable the bottom torus, causing the others to fall through and reveal
   that disabled deformables do not participate in contact. */
  plant.deformable_model().Disable(torus_ids[0], &plant_context);
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase enabling/disabling of deformable bodies."
      "Deformable torus bodies are stacked on top of each other and enabled "
      "one-by-one. Refer to README for instructions on meldis as well as "
      "optional flags.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
