#include "drake/common/test_utilities/maybe_pause_for_user.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_gflags.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/visualization/visualization_config_functions.h"

using namespace drake::multibody;
using namespace drake::geometry;
using namespace drake::math;
using namespace drake::systems;
using namespace drake::visualization;

namespace drake {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace examples {
namespace multibody {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

DEFINE_double(simulation_time, 5.0, "Duration of the simulation in seconds.");

DEFINE_double(dt, 1e-2, "Simulation time step [s].");
// Effectively infinite to force the constraint into the near-rigid regime.
DEFINE_double(k, 1e20, "Constraint stiffness [N/m].");
// Used to calculate tau_d, but will be ignored with infinite stiffness (tau_d
// set to dt).
DEFINE_double(d, 0, "Constraint dissipation [N⋅s/m]");
// Set lower limit to -inf so that the tendon considers a single constraint
// equation.
DEFINE_double(lower, -kInf, "Joint 0 lower limit [m].");
DEFINE_double(upper, 1.0, "Joint 0 upper limit [m].");

int do_main() {
  DiagramBuilder<double> builder;
  MultibodyPlantConfig config{
      .time_step = FLAGS_dt,
      .discrete_contact_approximation = "sap",
  };

  auto [plant, scene_graph] = AddMultibodyPlant(config, &builder);

  // Add a single body with a prismatic joint
  const double mass = 1.0;

  auto& body0 = plant.AddRigidBody(
      "block0",
      SpatialInertia<double>::MakeFromCentralInertia(
          mass, Eigen::Vector3d::Zero(), UnitInertia<double>::SolidCube(0.1)));

  auto& joint0 = plant.template AddJoint<PrismaticJoint>(
      "prismatic_joint0", plant.world_body(), std::nullopt, body0, std::nullopt,
      Eigen::Vector3d::UnitX());

  auto& actuator0 =
      plant.AddJointActuator("prismatic_joint0_actuator", joint0, 10);
  plant.get_mutable_joint_actuator(actuator0.index())
      .set_controller_gains({.p = 1e6, .d = 1e2});

  auto& body1 = plant.AddRigidBody(
      "block1",
      SpatialInertia<double>::MakeFromCentralInertia(
          mass, Eigen::Vector3d::Zero(), UnitInertia<double>::SolidCube(0.1)));

  auto& joint1 = plant.template AddJoint<PrismaticJoint>(
      "prismatic_joint1", plant.world_body(),
      RigidTransformd(Vector3d(0, 1, 0)), body1, std::nullopt,
      -Eigen::Vector3d::UnitX());
  unused(joint1);

  //   // Add the tendon constraint.
  //   std::vector<JointIndex> joints = {joint0.index(), joint1.index()};
  //   std::vector<double> a = {1.0, -2.0};

  //   // Coupler constraint between joint0 and joint2.
  //   plant.AddFixedTendonConstraint(joints, a, 0.0, 0.0, 0.0, FLAGS_k,
  //   FLAGS_d);

  // Joint limit constraint on joint0.
  plant.AddFixedTendonConstraint({joint0.index()}, {1.0}, 0.0, FLAGS_lower,
                                 FLAGS_upper, FLAGS_k, FLAGS_d);

  // Add visual geometry
  plant.RegisterVisualGeometry(body0, RigidTransformd::Identity(),
                               Box(0.1, 0.1, 0.1), "block0_visual",
                               Eigen::Vector4d(0.8, 0.2, 0.2, 1.0));
  plant.RegisterVisualGeometry(body1, RigidTransformd::Identity(),
                               Box(0.1, 0.1, 0.1), "block1_visual",
                               Eigen::Vector4d(0.2, 0.8, 0.2, 1.0));
  plant.Finalize();

  // Add visualization
  AddDefaultVisualization(&builder);

  auto diagram = builder.Build();
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  Context<double>& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());

  plant.get_desired_state_input_port(joint0.model_instance())
      .FixValue(&plant_context, Vector2d(FLAGS_upper, 0));

  // Create a simulator and run the simulation
  std::unique_ptr<Simulator<double>> simulator =
      MakeSimulatorFromGflags(*diagram, std::move(diagram_context));

  simulator->Initialize();
  common::MaybePauseForUser();
  simulator->AdvanceTo(FLAGS_simulation_time);

  plant.get_desired_state_input_port(joint0.model_instance())
      .FixValue(&plant_context, Vector2d(FLAGS_lower, 0));

  simulator->AdvanceTo(5 * FLAGS_simulation_time);

  return 0;
}
}  // namespace
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char** argv) {
  FLAGS_simulator_target_realtime_rate = 1.0;
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::do_main();
}
