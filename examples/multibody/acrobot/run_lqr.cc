#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_viewer_draw.hpp"
#include "drake/multibody/benchmarks/acrobot/make_acrobot_plant.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/serializer.h"
#include "drake/systems/primitives/affine_system.h"
#include "drake/systems/rendering/pose_bundle_to_draw_message.h"

namespace drake {

using geometry::SceneGraph;
using lcm::DrakeLcm;
using multibody::benchmarks::acrobot::AcrobotParameters;
using multibody::benchmarks::acrobot::MakeAcrobotPlant;
using multibody::multibody_plant::MultibodyPlant;
using multibody::RevoluteJoint;
using systems::Context;
using systems::lcm::LcmPublisherSystem;
using systems::lcm::Serializer;
using systems::rendering::PoseBundleToDrawMessage;

namespace examples {
namespace multibody {
namespace acrobot {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");

std::unique_ptr<systems::AffineSystem<double>> BalancingLQRController(
    const AcrobotParameters& acrobot_parameters) {
  std::unique_ptr<const MultibodyPlant<double>> acrobot =
      MakeAcrobotPlant(acrobot_parameters, true);
  const RevoluteJoint<double>& shoulder =
      acrobot->GetJointByName<RevoluteJoint>(
          acrobot_parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      acrobot->GetJointByName<RevoluteJoint>(
          acrobot_parameters.elbow_joint_name());
  std::unique_ptr<Context<double>> context = acrobot->CreateDefaultContext();

  // Set nominal actuation torque to zero.
  context->FixInputPort(0, Vector1d::Constant(0.0));

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
      *acrobot, *context, Q, R);
}

int do_main() {
  systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double simulation_time = FLAGS_simulation_time;

  // Make and add the acrobot model.
  const AcrobotParameters acrobot_parameters;
  const MultibodyPlant<double>& acrobot = *builder.AddSystem(MakeAcrobotPlant(
      acrobot_parameters, true /* Finalize the plant */, &scene_graph));
  const RevoluteJoint<double>& shoulder =
      acrobot.GetJointByName<RevoluteJoint>(
          acrobot_parameters.shoulder_joint_name());
  const RevoluteJoint<double>& elbow =
      acrobot.GetJointByName<RevoluteJoint>(
          acrobot_parameters.elbow_joint_name());

  // For this example the controller's model of the plant exactly matches the
  // plant to be controlled (in reality there would always be a mismatch).
  auto controller = builder.AddSystem(
      BalancingLQRController(acrobot_parameters));
  controller->set_name("controller");
  builder.Connect(acrobot.get_continuous_state_output_port(),
                  controller->get_input_port());
  builder.Connect(controller->get_output_port(),
                  acrobot.get_actuation_input_port());

  // Boilerplate used to connect the plant to a SceneGraph for
  // visualization.
  DrakeLcm lcm;
  const PoseBundleToDrawMessage& converter =
      *builder.template AddSystem<PoseBundleToDrawMessage>();
  LcmPublisherSystem& publisher =
      *builder.template AddSystem<LcmPublisherSystem>(
          "DRAKE_VIEWER_DRAW",
          std::make_unique<Serializer<drake::lcmt_viewer_draw>>(), &lcm);
  publisher.set_publish_period(1 / 60.0);

  // Sanity check on the availability of the optional source id before using it.
  DRAKE_DEMAND(!!acrobot.get_source_id());

  builder.Connect(
      acrobot.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(acrobot.get_source_id().value()));

  builder.Connect(scene_graph.get_pose_bundle_output_port(),
                  converter.get_input_port(0));
  builder.Connect(converter, publisher);

  // Last thing before building the diagram; dispatch the message to load
  // geometry.
  geometry::DispatchLoadMessage(scene_graph);

  // And build the Diagram:
  std::unique_ptr<systems::Diagram<double>> diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& acrobot_context =
      diagram->GetMutableSubsystemContext(acrobot, diagram_context.get());

  // Set an initial condition near the upright fixed point.
  shoulder.set_angle(&acrobot_context, M_PI + 0.1);
  shoulder.set_angular_rate(&acrobot_context, 0.0);
  elbow.set_angle(&acrobot_context, -0.1);
  elbow.set_angular_rate(&acrobot_context, 0.0);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(simulation_time);

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
  drake::logging::HandleSpdlogGflags();
  return drake::examples::multibody::acrobot::do_main();
}
