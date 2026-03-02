/* Test that CENIC is capable of resolving a weld constraint with a large
initial error, and then is able to maintain the constraint over time under
gravity.

Setup:
  - box1: a cube welded to the world at z=0.5 (no joint in MJCF).
  - box2: an cube, free body (6 DOF), connected to box1 via
    AddWeldConstraint. The constraint places box2's bottom face against
    box1's top face. box2 starts severely displaced so the ICF constraint
    correction required is large.
*/

#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "drake/math/rigid_transform.h"
#include "drake/multibody/cenic/cenic_integrator.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace icf {
namespace {

using Eigen::Vector3d;
using math::RigidTransformd;
using multibody::CenicIntegrator;
using multibody::MultibodyPlant;
using multibody::MultibodyPlantConfig;
using multibody::Parser;
using systems::DiagramBuilder;
using systems::Simulator;

// MJCF model defining two boxes and a floor.
//
// box1: half-extents 0.15m, welded to the world (no joint) at z=0.5.
// box2: half-extents 0.10m, free floating body (freejoint). The weld
//       constraint will be added programmatically below.
// floor: half-extents 1m x 1m x 0.05m, welded to world with top face at z=0.
constexpr char kMjcf[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="weld_constraint_demo">
    <worldbody>
      <!-- box1: welded to world at z=1.0 (no joint means fixed) -->
      <body name="box1" pos="0 0 1.0">
        <inertial mass="2" diaginertia="0.015 0.015 0.015"/>
        <geom type="box" size="0.15 0.15 0.15" rgba="0.5 0.5 0.5 1"/>
      </body>
      <!-- box2: free body, to be weld-constrained to box1 -->
      <body name="box2" pos="0 0 0.75">
        <joint name="box2_free" type="free"/>
        <inertial mass="1" diaginertia="0.0067 0.0067 0.0067"/>
        <geom type="box" size="0.1 0.1 0.1" rgba="0.9 0.4 0.1 1"/>
      </body>
      <!-- floor for visual context -->
      <geom name="floor" type="box" pos="0 0 -0.05" size="1 1 0.05"
            rgba="0.2 0.8 0.3 1"/>
    </worldbody>
  </mujoco>
)""";

GTEST_TEST(WeldConstraintSimulation, LargeInitialError) {
  DiagramBuilder<double> builder;

  // Continuous-time plant is required for the CENIC integrator.
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.0;
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  Parser(&plant).AddModelsFromString(kMjcf, "xml");

  const auto& box1 = plant.GetBodyByName("box1");
  const auto& box2 = plant.GetBodyByName("box2");

  // Weld constraint: frame P on box1 at the center of its top face (+z),
  // frame Q on box2 at the center of its bottom face (-z).
  // When satisfied, box2 sits on top of box1:
  //   box1 center at z=1.0, top face at z=1.15 → box2 center at z=1.25.
  const RigidTransformd X_box1_P(Vector3d(0.0, 0.0, -0.15));
  const RigidTransformd X_box2_Q(Vector3d(0.0, 0.0, 0.1));
  plant.AddWeldConstraint(box1, X_box1_P, box2, X_box2_Q);

  plant.Finalize();
  auto diagram = builder.Build();

  // Set initial conditions: box2's exact constrained position would be center
  // of box2 at (0, 0, 0.75), just below box1. Here we start with box2's center
  // at (0, 0, 0.75 - initial_gap), so CENIC must overcome the initial gap
  // error to pull box2 up into alignment. An initial_gap of 0.5m is a very
  // large error. This will cause CENIC to take many small steps to
  // converge, but it should still succeed (it did not originally). The weld
  // constraint resists gravity to hold box2 in place. Without the
  // constraint, box2 would fall freely under gravity.
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
  const double initial_gap = 0.5;
  plant.SetFloatingBaseBodyPoseInWorldFrame(
      &plant_context, box2,
      RigidTransformd(Vector3d(0.0, 0.0, 0.75 - initial_gap)));

  // Set up the simulator with the CENIC integrator.
  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  auto& integrator = simulator->reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);  // Use error control.
  integrator.set_target_accuracy(1e-3);

  simulator->Initialize();
  simulator->AdvanceTo(0.5);

  // Verify the weld constraint held: box2's center should be at z=0.75.
  // box1 center is at z=1.0. Frame P is at z=-0.15 on box1 (world z=0.85).
  // Frame Q is at z=+0.10 on box2, so box2 center = 0.85 - 0.10 = 0.75.
  const auto& final_plant_context =
      plant.GetMyContextFromRoot(simulator->get_context());
  const Vector3d& p_WB2 =
      plant.EvalBodyPoseInWorld(final_plant_context, box2).translation();
  const double kTolerance = 1e-5;  // 0.01 mm tolerance.
  const Vector3d expected_position(0.0, 0.0, 0.75);
  const double position_error = (p_WB2 - expected_position).norm();
  EXPECT_LE(position_error, kTolerance);
}

}  // namespace
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
