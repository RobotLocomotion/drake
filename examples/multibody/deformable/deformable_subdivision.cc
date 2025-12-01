#include <memory>
#include <utility>

#include <gflags/gflags.h>

#include "drake/examples/multibody/deformable/deformable_common.h"
#include "drake/examples/multibody/deformable/point_source_force_field.h"
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
DEFINE_string(contact_approximation, "lagged",
              "Type of convex contact approximation. See "
              "multibody::DiscreteContactApproximation for details. Options "
              "are: 'sap', 'lagged', and 'similar'.");
DEFINE_double(
    contact_damping, 10.0,
    "Hunt and Crossley damping for the deformable body, only used when "
    "'contact_approximation' is set to 'lagged' or 'similar' [s/m].");
DEFINE_int32(element_subdivision_count, 0,
             "The number of times each FEM element is subdivided when "
             "evaluating external forces.");

using drake::examples::deformable::PointSourceForceField;
using drake::geometry::IllustrationProperties;
using drake::geometry::Sphere;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlant;
using drake::multibody::DeformableBodyId;
using drake::multibody::DeformableModel;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::fem::DeformableBodyConfig;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::Vector4d;

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

  /* Set up a deformable body. We used fixed parameters here as they are not
   important for the demonstration. */
  DeformableBodyConfig<double> deformable_config;
  deformable_config.set_youngs_modulus(3e4);
  deformable_config.set_poissons_ratio(0.4);
  deformable_config.set_mass_density(1e3);
  deformable_config.set_stiffness_damping_coefficient(0.01);
  deformable_config.set_element_subdivision_count(
      FLAGS_element_subdivision_count);

  DeformableModel<double>& deformable_model = plant.mutable_deformable_model();

  /* Add a deformable torus body to the scene. */
  const double scale = 1.0;
  /* Minor diameter of the torus inferred from the vtk file. */
  const double kL = 0.09 * scale;

  /* Set the torus pose just above the floor surface. */
  const RigidTransformd X_WB(Vector3<double>(0.0, 0.0, kL / 2.0));
  RegisterDeformableTorus(&deformable_model, "deformable_torus", X_WB,
                          deformable_config, scale, FLAGS_contact_damping);

  /* Define a very concentrated force field which falls between the vertices of
   a coarse torus mesh. When the element subdivision is increased, the
   concentrated force is picked up by the more dense mesh, and the simulation
   accuracy improves. */
  Vector3d p_BC(0.07, 0.07, 0.09);
  double falloff_distance = 0.020;
  const PointSourceForceField* force_field_ptr{nullptr};
  auto force_field = std::make_unique<PointSourceForceField>(
      plant, plant.world_body(), p_BC, falloff_distance);
  force_field_ptr = force_field.get();
  plant.mutable_deformable_model().AddExternalForce(std::move(force_field));

  /* Add a visual hint for the force field source. */
  const Sphere sphere{falloff_distance};
  IllustrationProperties source_illustration_props;
  source_illustration_props.AddProperty("phong", "diffuse",
                                        Vector4d(0.1, 0.9, 0.1, 0.8));
  plant.RegisterVisualGeometry(plant.world_body(), RigidTransformd(p_BC),
                               sphere, "source_visual",
                               source_illustration_props);

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
  force_field_ptr->maximum_force_density_input_port().FixValue(&plant_context,
                                                               6e6);

  simulator.Initialize();
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);
  simulator.AdvanceTo(FLAGS_simulation_time);

  return 0;
}

}  // namespace
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "This is a demo used to showcase the subdivision of FEM elements. A "
      "coarse deformable torus is added to the scene and a very concentrated "
      "force field acts on it. At zero or low subdivision counts, the force "
      "field falls between the element vertices and has no effect. As the "
      "subdivision count increases, the simulation accuracy improves. Refer to "
      "README for instructions on meldis as well as optional flags.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::do_main();
}
