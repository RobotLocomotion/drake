#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/analysis/simulator_print_stats.h"
#include "drake/systems/controllers/pid_controller.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/matrix_gain.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/sine.h"

#include "drake/examples/multibody/bushing_as_revolute_joint/sim_utils.h"

DEFINE_double(simulation_time, 10.0,
              "Desired duration of the simulation in seconds.");
DEFINE_double(simulator_target_realtime_rate, 1.0, "Sim realtime rate.");

DEFINE_string(simulator_integration_scheme, "runge_kutta3", "integrator.");
DEFINE_double(simulator_max_time_step, 1e-3,
              "simulator max time step, for variable step integrators.");
DEFINE_double(simulator_accuracy, 1e-3,
              "simulator accuracy, for error controlled intergrators.");
DEFINE_bool(simulator_use_error_control, false, "Use error control?");
DEFINE_bool(simulator_publish_every_time_step, false,
            "publish every time step");
DEFINE_bool(publish_initial_frames, false, "publishes initial brick frames");
DEFINE_double(
    mbp_dt, 2.0e-4,
    "The fixed time step period (in seconds) of discrete updates for the "
    "multibody plant modeled as a discrete system. Strictly positive. "
    "Set to zero for a continuous plant model.");
DEFINE_string(drakeviz_role, "illustration", "Role for drake viz.");
DEFINE_double(initial_ySlider_joint_position, 0.0,
              "Initial ySlider_joint position value.");
DEFINE_bool(gravity_on, true, "Enable gravity?");

namespace drake {
namespace examples {
namespace multibody {
namespace bushing_as_revolute_joint {
namespace {

using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::math::RigidTransformd;
using drake::multibody::MultibodyPlant;

// This is the prismatic ySlider_joint's maximum value.
const double kMaxYSliderJointPosition = 0.1;

void SetInitialPoses(const MultibodyPlant<double>& plant,
                     double initial_ySlider_joint_position,
                     drake::systems::Context<double>* plant_context) {
  // Set the ySlider_body to its proper initial horizontal value.
  const drake::multibody::PrismaticJoint<double>& ySlider_body =
      plant.GetJointByName<drake::multibody::PrismaticJoint>("ySlider_joint");
  ySlider_body.set_translation(plant_context, -initial_ySlider_joint_position);

  const double ySlider_body_length = 0.3;
  const double y_initial_horizontal_distance =
      ySlider_body_length + initial_ySlider_joint_position;
  const double y_max_horizontal_distance =
      ySlider_body_length + kMaxYSliderJointPosition;
  const double b = 0.05;
  const double l2 =
      sqrt(std::pow(y_max_horizontal_distance, 2) + std::pow(b, 2));
  const double s = sqrt(std::pow(l2, 2) - std::pow(ySlider_body_length, 2));

  // Set the zSlider_body to its proper initial vertical value.
  const drake::multibody::PrismaticJoint<double>& zSlider_body =
      plant.GetJointByName<drake::multibody::PrismaticJoint>("zSlider_joint");
  DRAKE_DEMAND(initial_ySlider_joint_position <= kMaxYSliderJointPosition);
  const double slider_position =
      s - sqrt(std::pow(l2, 2) - std::pow(y_initial_horizontal_distance, 2));
  zSlider_body.set_translation(plant_context, slider_position);

  const drake::multibody::RevoluteJoint<double>& left_pushrod_pin =
      plant.GetJointByName<drake::multibody::RevoluteJoint>(
          "rod_revolute_joint");
  const double rod_revolute_joint_angle = asin(y_initial_horizontal_distance / l2);
  left_pushrod_pin.set_angle(plant_context, rod_revolute_joint_angle);
}

int do_main() {
  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  // Plant's parameters.
  const double g = FLAGS_gravity_on ? 9.81 : 0.0;  // m/s^2

  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(FLAGS_mbp_dt);

  drake::multibody::Parser parser(&plant);

  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  std::string file_name =
      "drake/examples/multibody/bushing_as_revolute_joint/transmission.sdf";
  std::string full_name = FindResourceOrThrow(file_name);
  const auto& id = parser.AddModelFromFile(full_name, "gripper");

  // The base frame is coincident with the world frame.
  const drake::multibody::Frame<double>& base_frame =
      plant.GetFrameByName("base");
  RigidTransformd X_WG = RigidTransformd::Identity();
  plant.WeldFrames(plant.world_frame(), base_frame, X_WG);

  // Gravity acting in the -z direction.
  plant.mutable_gravity_field().set_gravity_vector(-g *
                                                   Eigen::Vector3d::UnitZ());

  plant.Finalize();

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));

  drake::lcm::DrakeLcm lcm;
  drake::geometry::DrakeVisualizerParams viz_params;
  viz_params.role = FLAGS_drakeviz_role == "proximity" ? Role::kProximity
                                                       : Role::kIllustration;
  viz_params.publish_period = 1 / 30.0;
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, &lcm,
                                                  viz_params);
  ConnectContactResultsToDrakeVisualizer(&builder, plant);

  std::vector joint_vec{plant.GetJointByName("ySlider_joint").index()};
  auto Sx = plant.MakeStateSelectorMatrix(joint_vec);
  auto Sx_sys = builder.AddSystem<drake::systems::MatrixGain<double>>(Sx);
  builder.Connect(plant.get_state_output_port(), Sx_sys->get_input_port());

  drake::log()->info("num states: {}", plant.num_multibody_states(id));

  auto kp = drake::Vector1d(1000);
  auto kd = drake::Vector1d(200);
  auto ki = drake::Vector1d(0);
  auto pid_controller =
      builder.AddSystem<drake::systems::controllers::PidController<double>>(
          kp, ki, kd);

  builder.Connect(Sx_sys->get_output_port(),
                  pid_controller->get_input_port_estimated_state());

  DRAKE_DEMAND(FLAGS_initial_ySlider_joint_position <=
               kMaxYSliderJointPosition);
  const double phase =  M_PI_2 -
      M_PI * (FLAGS_initial_ySlider_joint_position / kMaxYSliderJointPosition);
  auto sine_sys = builder.AddSystem<drake::systems::Sine<double>>(
      kMaxYSliderJointPosition / 2.0, 2 * M_PI, phase, 1);
  auto adder_sys = builder.AddSystem<drake::systems::Adder<double>>(2, 1);
  auto const_src =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
          drake::Vector1d(-kMaxYSliderJointPosition / 2.0));
  builder.Connect(sine_sys->get_output_port(0), adder_sys->get_input_port(0));
  builder.Connect(const_src->get_output_port(), adder_sys->get_input_port(1));
  auto muxer = builder.AddSystem<drake::systems::Multiplexer<double>>(2);
  builder.Connect(adder_sys->get_output_port(), muxer->get_input_port(0));
  builder.Connect(sine_sys->get_output_port(1), muxer->get_input_port(1));
  builder.Connect(muxer->get_output_port(),
                  pid_controller->get_input_port_desired_state());

  builder.Connect(pid_controller->get_output_port_control(),
                  plant.get_actuation_input_port());

  auto frame_viz = builder.AddSystem<FrameViz>(plant, &lcm, 1 / 30.0);
  builder.Connect(plant.get_state_output_port(), frame_viz->get_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  drake::systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  SetInitialPoses(plant, FLAGS_initial_ySlider_joint_position, &plant_context);

  // PublishBodyFrames(plant_context, plant, &lcm);

  const drake::systems::SimulatorConfig config{
      FLAGS_simulator_integration_scheme,
      FLAGS_simulator_max_time_step,
      FLAGS_simulator_accuracy,
      FLAGS_simulator_use_error_control,
      FLAGS_simulator_target_realtime_rate,
      FLAGS_simulator_publish_every_time_step};

  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, std::move(diagram_context));
  drake::systems::ApplySimulatorConfig(simulator.get(), config);

  using clock = std::chrono::steady_clock;
  const clock::time_point start = clock::now();
  simulator->AdvanceTo(FLAGS_simulation_time);
  const clock::time_point end = clock::now();
  const double wall_clock_time =
      std::chrono::duration<double>(end - start).count();
  fmt::print("Simulator::AdvanceTo() wall clock time: {:.4g} seconds.\n",
             wall_clock_time);

  drake::systems::PrintSimulatorStatistics(*simulator);

  return 0;
}

}  // namespace
}  // bushing_as_revolute_joint
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bushing_as_revolute_joint::do_main();
}