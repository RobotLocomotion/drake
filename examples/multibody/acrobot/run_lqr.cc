#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/affine_system.h"

namespace drake {

using geometry::SceneGraph;
using lcm::DrakeLcm;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::MultibodyPlant;
using multibody::Parser;
using multibody::JointActuator;
using multibody::RevoluteJoint;
using systems::Context;
using systems::InputPort;
using Eigen::Vector2d;

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 3.0,
              "Desired duration of the simulation in seconds.");

DEFINE_bool(time_stepping, true, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");

// This helper method makes an LQR controller to balance an acrobot model
// specified in the SDF file `file_name`.
std::unique_ptr<systems::AffineSystem<double>> MakeBalancingLQRController(
    const std::string &file_name) {
  const std::string full_name = FindResourceOrThrow(file_name);
  // LinearQuadraticRegulator() below requires the controller's model of the
  // plant to only have a single input port corresponding to the actuation.
  // Therefore we create a new model that meets this requirement. (a model
  // created along with a SceneGraph for simulation would also have input ports
  // to interact with that SceneGraph).
  MultibodyPlant<double> acrobot(0.0);
  Parser parser(&acrobot);
  parser.AddModelFromFile(full_name);
  // We are done defining the model.
  acrobot.Finalize();

  const RevoluteJoint<double>& shoulder =
      acrobot.GetJointByName<RevoluteJoint>("ShoulderJoint");
  const RevoluteJoint<double>& elbow =
      acrobot.GetJointByName<RevoluteJoint>("ElbowJoint");
  std::unique_ptr<Context<double>> context = acrobot.CreateDefaultContext();

  // Set nominal actuation torque to zero.
  const InputPort<double>& actuation_port = acrobot.get_actuation_input_port();
  actuation_port.FixValue(context.get(), 0.0);
  acrobot.get_applied_generalized_force_input_port().FixValue(
      context.get(), Vector2d::Constant(0.0));

  shoulder.set_angle(context.get(), M_PI);
  shoulder.set_angular_rate(context.get(), 0.0);
  elbow.set_angle(context.get(), 0.0);
  elbow.set_angular_rate(context.get(), 0.0);

  // Setup LQR Cost matrices (penalize position error 10x more than velocity
  // to roughly address difference in units, using sqrt(g/l) as the time
  // constant.
  Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
  Q(0, 0) = 10;
  Q(1, 1) = 10;
  Vector1d R = Vector1d::Constant(1);

  return systems::controllers::LinearQuadraticRegulator(
      acrobot, *context, Q, R,
      Eigen::Matrix<double, 0, 0>::Zero() /* No cross state/control costs */,
      actuation_port.get_index());
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? 1.0e-3 : 0.0;

  // Make and add the acrobot model.
  const std::string relative_name =
      "drake/multibody/benchmarks/acrobot/acrobot.sdf";
  const std::string full_name = FindResourceOrThrow(relative_name);
  MultibodyPlant<double>& acrobot =
      *builder.AddSystem<MultibodyPlant>(time_step);

  Parser parser(&acrobot, &scene_graph);
  parser.AddModelFromFile(full_name);

  // We are done defining the model.
  acrobot.Finalize();

  DRAKE_DEMAND(acrobot.num_actuators() == 1);
  DRAKE_DEMAND(acrobot.num_actuated_dofs() == 1);

  RevoluteJoint<double>& shoulder =
      acrobot.GetMutableJointByName<RevoluteJoint>("ShoulderJoint");
  RevoluteJoint<double>& elbow =
      acrobot.GetMutableJointByName<RevoluteJoint>("ElbowJoint");

  // Drake's parser will default the name of the actuator to match the name of
  // the joint it actuates.
  const JointActuator<double>& actuator =
      acrobot.GetJointActuatorByName("ElbowJoint");
  DRAKE_DEMAND(actuator.joint().name() == "ElbowJoint");

  // For this example the controller's model of the plant exactly matches the
  // plant to be controlled (in reality there would always be a mismatch).
  auto controller = builder.AddSystem(
      MakeBalancingLQRController(relative_name));
  controller->set_name("controller");
  builder.Connect(acrobot.get_state_output_port(),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot.get_actuation_input_port());

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(acrobot.get_source_id().has_value());

  builder.Connect(
      acrobot.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(acrobot.get_source_id().value()));

  geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);

  RandomGenerator generator;

  // Setup distribution for random initial conditions.
  std::normal_distribution<symbolic::Expression> gaussian;
  shoulder.set_random_angle_distribution(M_PI + 0.02*gaussian(generator));
  elbow.set_random_angle_distribution(0.05*gaussian(generator));

  for (int i = 0; i < 5; i++) {
    simulator.get_mutable_context().SetTime(0.0);
    simulator.get_system().SetRandomContext(&simulator.get_mutable_context(),
                                            &generator);

    simulator.Initialize();
    simulator.AdvanceTo(FLAGS_simulation_time);
  }

  return 0;
}

}  // namespace
}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A simple acrobot demo using Drake's MultibodyPlant with "
      "LQR stabilization. "
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::acrobot::do_main();
}
