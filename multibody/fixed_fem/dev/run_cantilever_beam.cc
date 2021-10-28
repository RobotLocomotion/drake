/**
A demo to showcase FEM solver.

To run the demo. First ensure that you have the visualizer and the demo itself
built:

```
bazel build //tools:drake_visualizer
bazel build //multibody/fixed_fem/dev:run_cantilever_beam
```

Then, in one terminal, launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

In another terminal, launch the demo
```
bazel-bin/multibody/fixed_fem/dev/run_cantilever_beam
``` */
#include <memory>

#include <gflags/gflags.h>

#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_body_config.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"
#include "drake/multibody/fixed_fem/dev/deformable_visualizer.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

DEFINE_double(simulation_time, 10.0,
              "How many seconds to simulate the system.");
DEFINE_double(dx, 0.1,
              "Distance between consecutive vertices in the tet mesh, with "
              "unit m. Must be positive and smaller than or equal to 0.1 to "
              "provide enough resolution for reasonable accuracy. The smaller "
              "this number is, the higher the resolution of the mesh will be.");
DEFINE_double(E, 1e7,
              "Young's modulus of the deformable objects, with unit Pa");
DEFINE_double(nu, 0.4, "Poisson ratio of the deformable objects, unitless");
DEFINE_double(density, 1e4,
              "Mass density of the deformable objects, with unit kg/m³");
DEFINE_double(alpha, 0.001,
              "Mass damping coefficient. The damping ratio contributed by this "
              "coefficient is inversely proportional to the frequency of the "
              "motion. Note that mass damping damps out rigid body "
              "motion and thus this coefficient should be kept small. ");
DEFINE_double(
    beta, 0.001,
    "Stiffness damping coefficient. The damping ratio contributed by this "
    "coefficient is proportional to the frequency of the motion.");

namespace drake {
namespace multibody {
namespace fem {

int DoMain() {
  systems::DiagramBuilder<double> builder;
  const double dt = 1.0 / 60.0;
  DRAKE_DEMAND(FLAGS_dx > 0);
  const double dx = std::min(0.1, FLAGS_dx);
  MultibodyPlant<double>& plant = AddMultibodyPlantSceneGraph(&builder, dt);
  auto deformable_model = std::make_unique<DeformableModel<double>>(&plant);
  const geometry::Box box(1.5, 0.2, 0.2);
  /* A dummy proximity property that's used since there is no contact in this
   demo. */
  geometry::ProximityProperties dummy_proximity_props;
  geometry::AddContactMaterial({}, {},
                               multibody::CoulombFriction<double>(0, 0),
                               &dummy_proximity_props);

  /* Set up the corotated bar. */
  const math::RigidTransform<double> translation_left(
      Vector3<double>(0, -0.5, 0));
  DeformableBodyConfig<double> nonlinear_bar_config;
  nonlinear_bar_config.set_youngs_modulus(FLAGS_E);
  nonlinear_bar_config.set_poisson_ratio(FLAGS_nu);
  nonlinear_bar_config.set_mass_damping_coefficient(FLAGS_alpha);
  nonlinear_bar_config.set_stiffness_damping_coefficient(FLAGS_beta);
  nonlinear_bar_config.set_mass_density(FLAGS_density);
  nonlinear_bar_config.set_material_model(MaterialModel::kCorotated);
  const internal::ReferenceDeformableGeometry<double> nonlinear_bar_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(box, dx, translation_left);
  const DeformableBodyIndex nonlinear_bar_body_index =
      deformable_model->RegisterDeformableBody(
          nonlinear_bar_geometry, "Corotated", nonlinear_bar_config,
          dummy_proximity_props);

  /* Set up the linear bar. */
  DeformableBodyConfig<double> linear_bar_config(nonlinear_bar_config);
  linear_bar_config.set_material_model(MaterialModel::kLinear);
  const math::RigidTransform<double> translation_right(
      Vector3<double>(0, 0.5, 0));
  const internal::ReferenceDeformableGeometry<double> linear_bar_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(box, dx, translation_right);
  const DeformableBodyIndex linear_bar_body_index =
      deformable_model->RegisterDeformableBody(linear_bar_geometry, "Linear",
                                               linear_bar_config,
                                               dummy_proximity_props);

  /* Plug the two bars in to a wall. */
  const Vector3<double> wall_origin(-0.75, 0, 0);
  const Vector3<double> wall_normal(1, 0, 0);
  deformable_model->SetWallBoundaryCondition(nonlinear_bar_body_index,
                                             wall_origin, wall_normal);
  deformable_model->SetWallBoundaryCondition(linear_bar_body_index, wall_origin,
                                             wall_normal);

  std::vector<geometry::VolumeMesh<double>> reference_meshes;
  for (const internal::ReferenceDeformableGeometry<double>& geometry :
       deformable_model->reference_configuration_geometries()) {
    reference_meshes.emplace_back(geometry.mesh());
  }
  auto* visualizer = builder.AddSystem<DeformableVisualizer>(
      1.0 / 60.0, deformable_model->names(), reference_meshes);
  const DeformableModel<double>* deformable_model_ptr = deformable_model.get();
  plant.AddPhysicalModel(std::move(deformable_model));
  plant.Finalize();
  auto deformable_rigid_manager =
      std::make_unique<DeformableRigidManager<double>>(
          std::make_unique<
              multibody::contact_solvers::internal::PgsSolver<double>>());
  plant.SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
  builder.Connect(deformable_model_ptr->get_vertex_positions_output_port(),
                  visualizer->get_input_port());
  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();
  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(context));
  simulator->AdvanceTo(FLAGS_simulation_time);
  return 0;
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage("Demonstration of large deformation.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::fem::DoMain();
}
