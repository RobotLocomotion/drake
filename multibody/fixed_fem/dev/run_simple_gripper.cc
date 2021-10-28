/** @file

A demo to showcase integration of deformable solver and the contact solver.

In this demo, a force-controlled gripper with two prismatic joints (one that
controls the translation of the gripper and the other that controls one of the
fingers) grasps a deformable box with friction and squeezes it in a sinusoidal
motion. It is expected that the contact in stiction between the gripper and the
deformable box should render the whole system as, essentially, one body. To
verify that expectation, we apply a force to the gripper countering the effect
of gravity on the whole system (instead of anchoring the gripper) to provide
additional proof that the contact solver is working properly.

This demo can only be run in discrete mode. To run the demo. First ensure that
you have the visualizer and the demo itself built:

```
bazel build //tools:drake_visualizer
bazel build //multibody/fixed_fem/dev:run_simple_gripper
```

Then, in one terminal, launch the visualizer
```
bazel-bin/tools/drake_visualizer
```

In another terminal, launch the demo
```
bazel-bin/multibody/fixed_fem/dev/run_simple_gripper
``` */

#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/contact_solvers/pgs_solver.h"
#include "drake/multibody/fixed_fem/dev/deformable_body_config.h"
#include "drake/multibody/fixed_fem/dev/deformable_model.h"
#include "drake/multibody/fixed_fem/dev/deformable_rigid_manager.h"
#include "drake/multibody/fixed_fem/dev/deformable_visualizer.h"
#include "drake/multibody/fixed_fem/dev/mesh_utilities.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/sine.h"

DEFINE_double(simulation_time, 10.0, "Desired duration of the simulation [s].");
DEFINE_double(dt, 5.0e-3,
              "Discrete time step for the system [s]. Must be "
              "positive.");
DEFINE_double(E, 1e4, "Young's modulus of the deformable objects [Pa].");
DEFINE_double(nu, 0.4, "Poisson ratio of the deformable objects, unitless.");
DEFINE_double(density, 1e3, "Mass density of the deformable objects [kg/m³].");
DEFINE_double(
    mass_damping, 0.001,
    "Mass damping coefficient [1/s]. The damping ratio contributed by this "
    "coefficient is inversely proportional to the frequency of the motion. "
    "Note that mass damping damps out rigid body motion and thus this "
    "coefficient should be kept small.");
DEFINE_double(
    stiffness_damping, 0.002,
    "Stiffness damping coefficient [s]. The damping ratio contributed by "
    "this coefficient is proportional to the frequency of the motion.");
DEFINE_double(min_gripper_force, 2,
              "The minimum force in the harmonic oscillation carried out by "
              "the gripper [N]. Must be positive.");
DEFINE_double(max_gripper_force, 17,
              "The maximum force in the harmonic oscillation carried out by "
              "the gripper [N]. Must be greater than `min_gripper_force`.");
DEFINE_double(grip_frequency, 2.0,
              "The frequency of the harmonic oscillation forces carried out "
              "by the gripper [Hz].");

namespace drake {
namespace multibody {
namespace fem {

int DoMain() {
  systems::DiagramBuilder<double> builder;
  DRAKE_DEMAND(FLAGS_dt > 0);
  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_dt);

  /* Side length of the deformable box. */
  constexpr double kL = 0.06;
  const geometry::Box box(kL, kL, kL);

  /* Define the deformable box's geometry, physical properties, and position.
   The position has been pre-computed so that the box fits snugly in the
   gripper. */
  const math::RigidTransform<double> p_WB(Vector3<double>(0.03, 0.013, 0.06));
  DeformableBodyConfig<double> box_config;
  box_config.set_youngs_modulus(FLAGS_E);
  box_config.set_poisson_ratio(FLAGS_nu);
  box_config.set_mass_damping_coefficient(FLAGS_mass_damping);
  box_config.set_stiffness_damping_coefficient(FLAGS_stiffness_damping);
  box_config.set_mass_density(FLAGS_density);
  box_config.set_material_model(MaterialModel::kCorotated);
  constexpr int kNumSubdivision =
      3;  // Number of blocks to divide the box into.
  const internal::ReferenceDeformableGeometry<double> box_geometry =
      MakeDiamondCubicBoxDeformableGeometry<double>(box, kL / kNumSubdivision,
                                                    p_WB);

  /* Set up proximity properties for the deformable box. */
  const CoulombFriction<double> surface_friction(1.0, 1.0);
  geometry::ProximityProperties proximity_props;
  geometry::AddContactMaterial({}, {}, surface_friction, &proximity_props);

  /* Register the deformable box in the DeformableModel. */
  auto deformable_model = std::make_unique<DeformableModel<double>>(&plant);
  deformable_model->RegisterDeformableBody(box_geometry, "Corotated",
                                           box_config, proximity_props);
  const DeformableModel<double>* deformable_model_raw = deformable_model.get();
  plant.AddPhysicalModel(std::move(deformable_model));

  /* Set up a simple gripper. */
  Parser parser(&plant);
  std::string full_name =
      FindResourceOrThrow("drake/examples/simple_gripper/simple_gripper.sdf");
  parser.AddModelFromFile(full_name);
  /* Add collision geometries. */
  const math::RigidTransformd X_BG = math::RigidTransformd::Identity();
  const Body<double>& left_finger = plant.GetBodyByName("left_finger");
  const Body<double>& right_finger = plant.GetBodyByName("right_finger");
  /* The size of the finger is set to match the visual geometries in
   examples/simple_gripper/simple_gripper.sdf. */
  plant.RegisterCollisionGeometry(left_finger, X_BG,
                                  geometry::Box(0.007, 0.081, 0.028),
                                  "left_finger_collision", proximity_props);
  plant.RegisterCollisionGeometry(right_finger, X_BG,
                                  geometry::Box(0.007, 0.081, 0.028),
                                  "left_finger_collision", proximity_props);

  /* All rigid and deformable models have been added. Finalize the plant. */
  plant.Finalize();

  /* Set up an update manager to handle the discrete updates. */
  auto deformable_rigid_manager =
      std::make_unique<DeformableRigidManager<double>>(
          std::make_unique<contact_solvers::internal::PgsSolver<double>>());
  DeformableRigidManager<double>* deformable_rigid_manager_raw =
      deformable_rigid_manager.get();
  plant.SetDiscreteUpdateManager(std::move(deformable_rigid_manager));
  deformable_rigid_manager_raw->RegisterCollisionObjects(scene_graph);

  /* We use a force-controlled gripper to
    1. compensate for gravity of the entire system and verify that the force
       required to hold the system in place in the z-direction matches
       expectation, and
    2. "squeeze" the deformable box in the y-direction to show grasping with
       friction as well as deformation of deformable objects under external
       forces. */

  /* The total mass of the system =
     The mass of the gripper + the mass of the deformable box. */
  const Body<double>& gripper_body = plant.GetBodyByName("body");
  const double kGripperMass = gripper_body.get_default_mass() +
                              left_finger.get_default_mass() +
                              right_finger.get_default_mass();
  const double kTotalMass =
      kGripperMass + kL * kL * kL * FLAGS_density;  // [kg]
  const double g = 9.81;                            // [m/s²]
  /* The force magnitude in the positive z direction to compensate for gravity
   of the system. */
  const double kConstantZForce = kTotalMass * g;  // [N]

  DRAKE_DEMAND(FLAGS_max_gripper_force > FLAGS_min_gripper_force);
  DRAKE_DEMAND(FLAGS_min_gripper_force > 0);
  const double kAmplitude =
      (FLAGS_max_gripper_force - FLAGS_min_gripper_force) / 2.0;
  const double kOffset = kAmplitude + FLAGS_min_gripper_force;
  /* Here we are use the same Sine source to:
    1. Generate a horizontal harmonic forcing of the finger with the prescribed
       phase, amplitude and frequency.
    2. Impose a constant vertical force to hold up the gripper. */
  const Vector2<double> amplitudes(0, kAmplitude);
  const Vector2<double> frequencies(0.0, FLAGS_grip_frequency);
  const Vector2<double> phases(0, 3 * M_PI_2);  // Start with the minimum force.
  const auto& harmonic_force = *builder.AddSystem<systems::Sine<double>>(
      amplitudes, frequencies, phases);
  const auto& constant_force =
      *builder.AddSystem<systems::ConstantVectorSource<double>>(
          Vector2<double>(kConstantZForce, kOffset));
  const auto& adder = *builder.AddSystem<systems::Adder<double>>(2, 2);
  /* Add up the constant force and the harmonic force source and supply the sum
   to the actuation port. */
  builder.Connect(harmonic_force.get_output_port(0), adder.get_input_port(0));
  builder.Connect(constant_force.get_output_port(), adder.get_input_port(1));
  builder.Connect(adder.get_output_port(0), plant.get_actuation_input_port());

  /* Set up visualizers. */
  std::vector<geometry::VolumeMesh<double>> reference_meshes;
  for (const internal::ReferenceDeformableGeometry<double>& geometry :
       deformable_model_raw->reference_configuration_geometries()) {
    reference_meshes.emplace_back(geometry.mesh());
  }
  auto& visualizer = *builder.AddSystem<DeformableVisualizer>(
      1.0 / 64.0, deformable_model_raw->names(), reference_meshes);
  builder.Connect(deformable_model_raw->get_vertex_positions_output_port(),
                  visualizer.get_input_port());
  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /* Set initial conditions for the gripper. */
  auto& plant_context =
      diagram->GetMutableSubsystemContext(plant, context.get());
  const PrismaticJoint<double>& finger_slider =
      plant.GetJointByName<PrismaticJoint>("finger_sliding_joint");
  /* Set the initial position of the gripper to be of the same width as the
   deformable box. */
  finger_slider.set_translation(&plant_context, -kL);
  finger_slider.set_translation_rate(&plant_context, 0);
  const PrismaticJoint<double>& translate_joint =
      plant.GetJointByName<PrismaticJoint>("translate_joint");
  translate_joint.set_translation(&plant_context, 0.0);
  translate_joint.set_translation_rate(&plant_context, 0.0);

  /* Build the simulator and run! */
  auto simulator =
      systems::MakeSimulatorFromGflags(*diagram, std::move(context));
  simulator->AdvanceTo(FLAGS_simulation_time);

  return 0;
}
}  // namespace fem
}  // namespace multibody
}  // namespace drake

int main(int argc, char** argv) {
  gflags::SetUsageMessage(
      "Demonstration of contact solver working with deformable objects.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::multibody::fem::DoMain();
}
