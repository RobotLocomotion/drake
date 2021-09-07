#include <math.h>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/examples/multibody/bushing_as_revolute_joint/sim_utils.h"
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

DEFINE_double(simulation_time, 2.0,
              "Desired duration of the simulation in seconds.");
DEFINE_double(simulator_target_realtime_rate, 1.0, "Sim realtime rate.");

DEFINE_string(simulator_integration_scheme, "runge_kutta3", "integrator.");
DEFINE_double(simulator_max_time_step, 1e-3,
              "Simulator max time step, for variable step integrators.");
DEFINE_double(simulator_accuracy, 1e-3,
              "Simulator accuracy, for error controlled intergrators.");
DEFINE_bool(simulator_use_error_control, false, "Use error control?");
DEFINE_bool(simulator_publish_every_time_step, false,
            "Publish every time step.");
DEFINE_bool(publish_initial_frames, false, "Publishes initial frames.");
DEFINE_double(mbp_dt, 2.0e-4,
              "Fixed time step period (in seconds) of discrete updates for the "
              "multibody plant modeled as a discrete system. Strictly positive."
              " Set to zero for a continuous plant model.");
DEFINE_string(drakeviz_role, "Illustration", "Role for drake vizualizer.");
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

void SetInitialPoses(const MultibodyPlant<double>& plant,
                     drake::systems::Context<double>* plant_context) {
  // Set the ySlider_body to an initial horizontal value of 0.0.
  const double initial_ySlider_joint_value = 0.0;
  const drake::multibody::PrismaticJoint<double>& ySlider_body =
      plant.GetJointByName<drake::multibody::PrismaticJoint>("ySlider_joint");
  ySlider_body.set_translation(plant_context, initial_ySlider_joint_value);

  // Set the zSlider_body to an initial vertical value that is consistent with
  // system geometry, initial_ySlider_joint_value, and an assembled bushing.
  const double initial_zSlider_joint_value = 0.0;
  const drake::multibody::PrismaticJoint<double>& zSlider_body =
      plant.GetJointByName<drake::multibody::PrismaticJoint>("zSlider_joint");
  zSlider_body.set_translation(plant_context, initial_zSlider_joint_value);

  // Set the rod_revolute_joint to its proper initial angle.
  const double ySlider_body_length = 0.3;
  const double rod_body_length = 0.5;
  const double rod_revolute_joint_angle =
      asin(ySlider_body_length / rod_body_length) + 0.2;
  const drake::multibody::RevoluteJoint<double>& rod_revolute_joint =
      plant.GetJointByName<drake::multibody::RevoluteJoint>(
          "rod_revolute_joint");
  rod_revolute_joint.set_angle(plant_context, rod_revolute_joint_angle);
}

int do_main() {
  drake::systems::DiagramBuilder<double> builder;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

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

  // Gravity acts in the -z direction.
  const double g = FLAGS_gravity_on ? 9.81 : 0.0;  // m/s^2
  const Eigen::Vector3d gravity_vector = -g * Eigen::Vector3d::UnitZ();
  plant.mutable_gravity_field().set_gravity_vector(gravity_vector);

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

  // Set the prismatic ySlider_joint's amplitude, frequency, phase, etc.
  // ySlider_joint_position = Amp * sin( omega * t + phase ).
  // ySlider_joint_velocity = Amp * cos( omega * t + phase ) * omega.
  // Since phase = pi/2, initially (t = 0), ySlider_joint_velocity = 0.
  const double kMaxYSliderJointPosition = 0.1;
  const double sine_amplitude = kMaxYSliderJointPosition / 2.0;
  const double sine_frequency =  2 * M_PI;
  const double phase =  M_PI_2;
  const int number_elements_in_output_signal = 1;
  auto sine_sys = builder.AddSystem<drake::systems::Sine<double>>(
      sine_amplitude, sine_frequency, phase, number_elements_in_output_signal);
  auto adder_sys = builder.AddSystem<drake::systems::Adder<double>>(2, 1);
  auto const_src =
      builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
          drake::Vector1d(-sine_amplitude));
  builder.Connect(sine_sys->get_output_port(0), adder_sys->get_input_port(0));
  builder.Connect(const_src->get_output_port(), adder_sys->get_input_port(1));
  auto muxer = builder.AddSystem<drake::systems::Multiplexer<double>>(2);
  builder.Connect(adder_sys->get_output_port(), muxer->get_input_port(0));
  builder.Connect(sine_sys->get_output_port(1), muxer->get_input_port(1));
  builder.Connect(muxer->get_output_port(),
                  pid_controller->get_input_port_desired_state());
  builder.Connect(pid_controller->get_output_port_control(),
                  plant.get_actuation_input_port());

  const double visualizer_period = 1.0 / 30.0;  // In units of seconds.
  auto frame_viz = builder.AddSystem<FrameViz>(plant, &lcm, visualizer_period);
  builder.Connect(plant.get_state_output_port(), frame_viz->get_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  drake::systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  SetInitialPoses(plant, &plant_context);

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
}  // namespace bushing_as_revolute_joint
}  // namespace multibody
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::multibody::bushing_as_revolute_joint::do_main();
}
