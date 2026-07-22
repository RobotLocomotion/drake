/* Tests that CENIC can resolve a distance constraint with a large initial
error and then maintain it over time under gravity, for both a rigid distance
constraint and a compliant (spring-damper) one.

Setup:
  - box1: a cube welded to the world at z=1.0 (no joint in MJCF), so point P at
    its bottom-face center is fixed at world (0, 0, 0.85).
  - box2: a cube, free body (6 DOF), connected to box1 via AddDistanceConstraint
    with point Q at box2's center of mass. The constraint holds ‖p_WQ − p_WP‖
    at the free length ℓ (rigid) or as a spring-damper (compliant). box2 starts
    severely displaced (a large initial distance error), so the ICF constraint
    correction required is large, which exercises the near-rigid small-time-step
    regime. Because Q is at box2's center of mass, gravity produces no moment
    and box2 hangs straight below P.
*/

#include <limits>
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
// box1: half-extents 0.15m, welded to the world (no joint) at z=1.0.
// box2: half-extents 0.10m, free floating body (freejoint). The distance
//       constraint will be added programmatically below.
// floor: half-extents 1m x 1m x 0.05m, for visual context.
constexpr char kMjcf[] = R"""(
  <?xml version="1.0"?>
  <mujoco model="distance_constraint_demo">
    <worldbody>
      <!-- box1: welded to world at z=1.0 (no joint means fixed) -->
      <body name="box1" pos="0 0 1.0">
        <inertial mass="2" diaginertia="0.015 0.015 0.015"/>
        <geom type="box" size="0.15 0.15 0.15" rgba="0.5 0.5 0.5 1"/>
      </body>
      <!-- box2: free body, to be distance-constrained to box1 -->
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

// Point P on box1 (bottom-face center) → world (0, 0, 0.85), and point Q at
// box2's center of mass (its body origin).
constexpr double kFreeLength = 0.3;
const Vector3d kPWorld(0.0, 0.0, 0.85);

// Builds the diagram/plant with a distance constraint (rigid iff stiffness is
// infinite), starts box2 with a large initial distance error, simulates with
// CENIC, and returns the final distance ‖p_WQ − p_WP‖.
double SimulateAndGetFinalDistance(double stiffness, double damping) {
  DiagramBuilder<double> builder;

  // Continuous-time plant is required for the CENIC integrator.
  MultibodyPlantConfig plant_config;
  plant_config.time_step = 0.0;
  auto [plant, scene_graph] = AddMultibodyPlant(plant_config, &builder);

  Parser(&plant).AddModelsFromString(kMjcf, "xml");

  const auto& box1 = plant.GetBodyByName("box1");
  const auto& box2 = plant.GetBodyByName("box2");

  const Vector3d p_box1_P(0.0, 0.0, -0.15);  // Bottom face of box1.
  const Vector3d p_box2_Q(0.0, 0.0, 0.0);    // box2 center of mass.
  plant.AddDistanceConstraint(box1, p_box1_P, box2, p_box2_Q, kFreeLength,
                              stiffness, damping);

  plant.Finalize();
  auto diagram = builder.Build();

  // Start box2 well below its constrained position: the constrained center is
  // at distance ~kFreeLength below P (i.e. z ≈ 0.55). Start at z = 0.05, so the
  // initial distance is 0.8 — a large error CENIC must resolve.
  auto context = diagram->CreateDefaultContext();
  auto& plant_context = plant.GetMyMutableContextFromRoot(context.get());
  plant.SetFloatingBaseBodyPoseInWorldFrame(
      &plant_context, box2, RigidTransformd(Vector3d(0.0, 0.0, 0.05)));

  auto simulator =
      std::make_unique<Simulator<double>>(*diagram, std::move(context));
  auto& integrator = simulator->reset_integrator<CenicIntegrator<double>>();
  integrator.set_maximum_step_size(0.1);
  integrator.set_fixed_step_mode(false);  // Use error control.
  integrator.set_target_accuracy(1e-3);

  simulator->Initialize();
  simulator->AdvanceTo(1.0);

  const auto& final_plant_context =
      plant.GetMyContextFromRoot(simulator->get_context());
  const RigidTransformd& X_WB2 =
      plant.EvalBodyPoseInWorld(final_plant_context, box2);
  const Vector3d p_WQ = X_WB2 * p_box2_Q;
  return (p_WQ - kPWorld).norm();
}

// A rigid distance constraint should drive the distance to the free length.
GTEST_TEST(DistanceConstraintSimulation, RigidLargeInitialError) {
  const double distance = SimulateAndGetFinalDistance(
      /*stiffness=*/std::numeric_limits<double>::infinity(), /*damping=*/0.0);
  // The near-rigid regularization allows a tiny stretch under gravity; 1e-3 is
  // comfortably tight (< 1 mm) while robust to that residual compliance.
  EXPECT_NEAR(distance, kFreeLength, 1e-3);
}

// A compliant distance constraint behaves as a spring-damper. At rest under
// gravity the spring stretches so that k⋅(d − ℓ) = m⋅g, i.e. d = ℓ + m⋅g/k.
GTEST_TEST(DistanceConstraintSimulation, CompliantSpring) {
  const double kStiffness = 200.0;  // N/m.
  const double kDamping = 20.0;     // N⋅s/m (settles within the sim horizon).
  const double kMass = 1.0;         // box2 mass, from the MJCF.
  const double kGravity = 9.81;     // Default plant gravity magnitude.

  const double distance = SimulateAndGetFinalDistance(kStiffness, kDamping);
  const double expected_distance = kFreeLength + kMass * kGravity / kStiffness;
  EXPECT_NEAR(distance, expected_distance, 2e-3);
}

}  // namespace
}  // namespace icf
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
