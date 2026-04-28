#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/geometry/meshcat.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/fem/deformable_body_config.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/deformable_model.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"

namespace drake {
namespace examples {
namespace conveyor_belt {
namespace {

DEFINE_double(max_time_step, 1e-3, "Simulation time step used for integrator.");

int do_main_continous_plant() {
  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_max_time_step);
  std::string conveyor_belt_url =
      "package://drake/examples/conveyor_belt/conveyor_belt.sdf";
  multibody::Parser(&builder).AddModelsFromUrl(conveyor_belt_url);

  // Register a deformable sphere to drop onto the conveyor belt.
  // The belt center is at z=1 with thickness 0.1, so its top surface is ~z=1.05.
  // We place the sphere center at z=1.4 (radius=0.15, so it starts ~0.2m above
  // the belt).
  if (FLAGS_max_time_step > 0) {
    // Switch to SAP, which is required for deformable bodies.
    plant.set_discrete_contact_approximation(
        multibody::DiscreteContactApproximation::kSap);

    multibody::fem::DeformableBodyConfig<double> sphere_config;
    sphere_config.set_youngs_modulus(3e4);
    sphere_config.set_poissons_ratio(0.4);
    sphere_config.set_mass_density(1e3);
    sphere_config.set_stiffness_damping_coefficient(0.01);

    geometry::ProximityProperties deformable_proximity_props;
    geometry::AddContactMaterial(
        /*dissipation=*/{}, /*point_stiffness=*/{},
        multibody::CoulombFriction<double>(0.8, 0.8),
        &deformable_proximity_props);

    const double kSphereRadius = 0.15;
    const double kResolutionHint = 0.1;
    auto sphere_instance = std::make_unique<geometry::GeometryInstance>(
        math::RigidTransformd(Eigen::Vector3d(0.0, 0.0, 1.4)),
        geometry::Sphere(kSphereRadius), "deformable_sphere");
    sphere_instance->set_proximity_properties(deformable_proximity_props);

    geometry::IllustrationProperties illustration_props;
    illustration_props.AddProperty("phong", "diffuse",
                                   Eigen::Vector4d(0.2, 0.8, 0.3, 1.0));
    sphere_instance->set_illustration_properties(illustration_props);

    plant.mutable_deformable_model().RegisterDeformableBody(
        std::move(sphere_instance), sphere_config, kResolutionHint);
  } else {
    fmt::print(
        "MultibodyPlant is running in continuous mode, so no deformable "
        "body will be registered.");
  }

  plant.Finalize();

  // Set up visualization
  auto meshcat = std::make_shared<geometry::Meshcat>();
  visualization::ApplyVisualizationConfig(
      visualization::VisualizationConfig{
          .default_proximity_color = geometry::Rgba{1, 0, 0, 0.25},
          .enable_alpha_sliders = true,
      },
      &builder, nullptr, nullptr, nullptr, meshcat);

  // Set up context
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();
  std::unique_ptr<systems::Context<double>> context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(context.get());

  // Force visualization
  diagram->ForcedPublish(*context);

  // Set up simulator
  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  meshcat->StartRecording();
  simulator.AdvanceTo(20.0);
  meshcat->PublishRecording();

  return 0;
}

}  // namespace
}  // namespace conveyor_belt
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  // Initialize gflags.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::examples::conveyor_belt::do_main_continous_plant();
  return 0;
}
