/// @file
/// Interactive demo of multibody::meshcat::MeshcatMouseSpring.
///
/// Drops a row of hydroelastic-contact boxes of varying size onto the ground
/// and simulates in real time. Open the printed Meshcat URL in a browser and
/// hold Ctrl while dragging a box with the left mouse button to pull it around.
/// Because the spring force is mass-scaled, all the boxes respond similarly.
///
/// The plant is stepped discretely by default. Pass --integrator=cenic to use
/// the CENIC/ICF integrator instead (which forces a continuous-time plant).

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include <gflags/gflags.h>

#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/shape_specification.h"
#include "drake/multibody/meshcat/meshcat_mouse_spring.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_string(integrator, "runge_kutta3",
              "Integration scheme (see SimulatorConfig). Use 'cenic' for the "
              "CENIC/ICF integrator; ignored while --time_step > 0.");
DEFINE_double(
    time_step, 1.0e-2,
    "Discrete MultibodyPlant time step, in s; forced to 0 (continuous "
    "plant) when --integrator=cenic.");
DEFINE_double(accuracy, 1.0e-2,
              "Integrator accuracy (for error-controlled schemes like cenic).");
DEFINE_double(stiffness, 100.0,
              "Mass-normalized drag-spring stiffness, in 1/s².");
DEFINE_double(realtime_rate, 1.0, "Target realtime rate for the simulation.");

namespace drake {
namespace examples {
namespace meshcat_mouse_spring {
namespace {

using Eigen::Vector3d;
using Eigen::Vector4d;
using geometry::Box;
using geometry::MeshcatVisualizer;
using geometry::ProximityProperties;
using math::RigidTransformd;
using multibody::ContactModel;
using multibody::CoulombFriction;
using multibody::MultibodyPlant;
using multibody::RigidBody;
using multibody::SpatialInertia;
using multibody::meshcat::MeshcatMouseSpring;

// Registers `box` on `body` as compliant hydroelastic collision geometry plus
// matching visual geometry of the given color.
void AddHydroelasticBox(MultibodyPlant<double>* plant,
                        const RigidBody<double>& body, const Box& box,
                        const std::string& name,
                        const CoulombFriction<double>& friction,
                        const Vector4d& color) {
  ProximityProperties props;
  geometry::AddContactMaterial(/* dissipation */ {}, /* point_stiffness */ {},
                               friction, &props);
  geometry::AddCompliantHydroelasticProperties(
      /* resolution_hint */ 0.05, /* hydroelastic_modulus */ 1.0e5, &props);
  plant->RegisterCollisionGeometry(body, RigidTransformd(), box,
                                   name + "_collision", std::move(props));
  plant->RegisterVisualGeometry(body, RigidTransformd(), box, name + "_visual",
                                color);
}

int DoMain() {
  const bool use_cenic = FLAGS_integrator == "cenic";
  const double time_step = use_cenic ? 0.0 : FLAGS_time_step;

  systems::DiagramBuilder<double> builder;
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, time_step);
  plant.set_contact_model(ContactModel::kHydroelastic);

  const CoulombFriction<double> friction(0.6, 0.5);

  // Ground: a rigid hydroelastic half space at z = 0 (not visualized).
  ProximityProperties ground_props;
  geometry::AddContactMaterial({}, {}, friction, &ground_props);
  geometry::AddRigidHydroelasticProperties(&ground_props);
  plant.RegisterCollisionGeometry(plant.world_body(), RigidTransformd(),
                                  geometry::HalfSpace(), "ground",
                                  std::move(ground_props));

  // A row of free compliant boxes of increasing size.
  const Vector4d colors[] = {{0.9, 0.2, 0.2, 1.0},
                             {0.2, 0.7, 0.3, 1.0},
                             {0.2, 0.4, 0.9, 1.0},
                             {0.9, 0.7, 0.2, 1.0},
                             {0.7, 0.3, 0.8, 1.0}};
  const int num_boxes = 5;
  for (int i = 0; i < num_boxes; ++i) {
    const double s = 0.01 + 0.04 * i;  // Edge length, from 1 cm to 17 cm.
    const std::string name = "box_" + std::to_string(i);
    const RigidBody<double>& body = plant.AddRigidBody(
        name, SpatialInertia<double>::SolidBoxWithDensity(1000.0, s, s, s));
    AddHydroelasticBox(&plant, body, Box(s, s, s), name, friction, colors[i]);
  }
  plant.Finalize();

  auto meshcat = std::make_shared<geometry::Meshcat>();
  MeshcatVisualizer<double>::AddToBuilder(&builder, scene_graph, meshcat);
  MeshcatMouseSpring<double>::AddToBuilder(&builder, &plant, meshcat,
                                           FLAGS_stiffness);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Place the boxes in a row above the ground.
  auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
  for (int i = 0; i < num_boxes; ++i) {
    const double x = (i - (num_boxes - 1) / 2.0) * 0.4;
    plant.SetFreeBodyPose(&plant_context,
                          plant.GetBodyByName("box_" + std::to_string(i)),
                          RigidTransformd(Vector3d(x, 0.0, 0.2)));
  }

  systems::Simulator<double> simulator(*diagram, std::move(context));
  systems::SimulatorConfig sim_config;
  sim_config.integration_scheme = FLAGS_integrator;
  sim_config.accuracy = FLAGS_accuracy;
  sim_config.target_realtime_rate = FLAGS_realtime_rate;
  systems::ApplySimulatorConfig(sim_config, &simulator);

  // Run until the "Stop" button is clicked in the Meshcat controls.
  meshcat->AddButton("Stop Simulation", "Escape");
  simulator.set_monitor([&meshcat, diagram = diagram.get()](
                            const systems::Context<double>&) {
    return meshcat->GetButtonClicks("Stop Simulation") > 0
               ? systems::EventStatus::ReachedTermination(diagram, "stopped")
               : systems::EventStatus::DidNothing();
  });
  simulator.Initialize();

  drake::log()->info(
      "Meshcat is available at {}. Hold Ctrl and drag a box with the left "
      "mouse button. Click 'Stop Simulation' (or press Escape) to exit.",
      meshcat->web_url());

  simulator.AdvanceTo(std::numeric_limits<double>::infinity());
  meshcat->DeleteButton("Stop Simulation");
  return 0;
}

}  // namespace
}  // namespace meshcat_mouse_spring
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::meshcat_mouse_spring::DoMain();
}
