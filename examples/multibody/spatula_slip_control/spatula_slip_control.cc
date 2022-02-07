#include <gflags/gflags.h>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant_config_functions.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/analysis/simulator_config_functions.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/adder.h"
#include "drake/systems/primitives/constant_vector_source.h"

// Parameters for squeezing the spatula.
DEFINE_double(gripper_force, 1, "The force to be applied by the gripper. [N].");
DEFINE_double(amplitude, 5,
              "The amplitude of the oscillations "
              "carried out by the gripper. [N].");
DEFINE_double(frequency, 2,
              "The frequency of the oscillations "
              "carried out by the gripper. [Hz].");
DEFINE_double(pulse_width, 4.5, "Pulse width of the control signal. [s].");
DEFINE_double(period, 6, "Period of the control signal. [s].");

// DrakeVisualizer Settings.
DEFINE_bool(visualize_collision, false,
            "Visualize collision instead of visual geom");

// MultibodyPlant settings.
DEFINE_double(stiction_tolerance, 1e-5, "Default stiction tolerance. [m/s].");
DEFINE_double(mbp_discrete_update_period, 1.0e-2,
              "If zero, the plant is modeled as a continuous system. "
              "If positive, the period (in seconds) of the discrete updates "
              "for the plant modeled as a discrete system."
              "This parameter must be non-negative.");
DEFINE_string(contact_model, "hydroelastic",
              "Contact model. Options are: 'point', 'hydroelastic', "
              "'hydroelastic_with_fallback'.");
DEFINE_string(contact_surface_representation, "polygon",
              "Contact-surface representation for hydroelastics. "
              "Options are: 'triangle' or 'polygon'. Default is 'polygon'.");

// Simulator settings.
DEFINE_double(realtime_rate, 1,
              "Desired rate of the simulation compared to realtime."
              "A value of 1 indicates real time.");
DEFINE_double(simulation_sec, 30, "Number of seconds to simulate. [s].");
DEFINE_double(accuracy, 1.0e-3, "The integration accuracy.");
DEFINE_double(max_time_step, 1.0e-2,
              "The maximum time step the integrator is allowed to take, [s].");
DEFINE_string(integration_scheme, "implicit_euler",
              "Integration scheme to be used. Available options are: "
              "'semi_explicit_euler','runge_kutta2','runge_kutta3',"
              "'implicit_euler'");

using drake::geometry::SceneGraph;
using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::multibody::MultibodyPlant;
using drake::multibody::MultibodyPlantConfig;
using drake::multibody::PrismaticJoint;
using drake::systems::ApplySimulatorConfig;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::SimulatorConfig;

namespace drake {
namespace examples {
namespace spatula_slip_control {
namespace {

// We create a simple leaf system that outputs a square wave signal for our
// open loop controller. The Square system here supports an arbitraritly
// dimensional signal, but we will use a 2-dimensional signal for our gripper.
class Square final : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Square)

  /// Constructs a %Square system where the amplitude, frequency, and phase is
  /// applied to every input.
  ///
  /// @param[in] amplitude  the square wave amplitude.
  /// @param[in] pulse_width the square wave pulse width.
  /// @param[in] period the square wave frequency. (radians/second)
  /// @param[in] phase the square wave phase. (radians)
  /// @param[in] size number of elements in the output signal.
  /// @param[in] is_time_based indicates whether to use the simulation time as
  ///            the source for the square wave time variable, or use an
  ///            external source, in which case an input port of size @p size is
  ///            created.
  Square(double amplitude, double pulse_width, double period, double phase,
         int size, bool is_time_based = true)
      : Square(Eigen::VectorXd::Ones(size) * amplitude,
               Eigen::VectorXd::Ones(size) * pulse_width,
               Eigen::VectorXd::Ones(size) * period,
               Eigen::VectorXd::Ones(size) * phase, is_time_based) {}

  /// Constructs a %Square system where different amplitudes, frequencies, and
  /// phases can be applied to each square wave.
  ///
  /// @param[in] amplitudes the square wave amplitudes.
  /// @param[in] pulse_widths the square wave pulse widths.
  /// @param[in] periods the square wave frequencies. (radians/second)
  /// @param[in] phases the square wave phases. (radians)
  /// @param[in] is_time_based indicates whether to use the simulation time as
  ///            the source for the square wave time variable, or use an
  ///            external source, in which case an input port is created.
  explicit Square(const Eigen::VectorXd& amplitudes,
                  const Eigen::VectorXd& pulse_widths,
                  const Eigen::VectorXd& periods, const Eigen::VectorXd& phases,
                  bool is_time_based = true)
      : amplitude_(amplitudes),
        pulse_width_(pulse_widths),
        period_(periods),
        phase_(phases),
        is_time_based_(is_time_based) {
    // Ensure the incoming vectors are all the same size.
    DRAKE_THROW_UNLESS(pulse_widths.size() == amplitudes.size());
    DRAKE_THROW_UNLESS(pulse_widths.size() == periods.size());
    DRAKE_THROW_UNLESS(pulse_widths.size() == phases.size());

    // Check each of the incoming vectors. For each vector, set a flag if every
    // element in that vector is the same.
    is_const_amplitude_ = amplitude_.isConstant(amplitude_[0]);
    is_const_pulse_width_ = pulse_width_.isConstant(pulse_width_[0]);
    is_const_period_ = period_.isConstant(period_[0]);
    is_const_phase_ = phase_.isConstant(phase_[0]);

    // If the Square system is system time based, do not create an input port.
    // System time is used as the time variable in this case. If the Square
    // system is not system time based, create an input port that contains the
    // signal to be used as the time variable.
    if (!is_time_based) {
      this->DeclareInputPort(systems::kUseDefaultName, systems::kVectorValued,
                             pulse_widths.size());
    }
    value_output_port_index_ =
        this->DeclareVectorOutputPort(systems::kUseDefaultName,
                                      pulse_widths.size(),
                                      &Square::CalcValueOutput)
            .get_index();
  }

  double amplitude() const {
    if (!is_const_amplitude_) {
      std::stringstream s;
      s << "The amplitude vector, [" << amplitude_
        << "], cannot be represented "
        << "as a scalar value. Please use "
        << "drake::systems::Square::amplitude_vector() instead.";
      throw std::logic_error(s.str());
    }
    return amplitude_[0];
  }

  double pulse_width() const {
    if (!is_const_pulse_width_) {
      std::stringstream s;
      s << "The pulse_width vector, [" << pulse_width_
        << "], cannot be represented "
        << "as a scalar value. Please use "
        << "drake::systems::Square::pulse_width_vector() instead.";
      throw std::logic_error(s.str());
    }
    return pulse_width_[0];
  }

  double period() const {
    if (!is_const_period_) {
      std::stringstream s;
      s << "The period vector, [" << period_ << "], cannot be represented "
        << "as a scalar value. Please use "
        << "drake::systems::Square::period_vector() instead.";
      throw std::logic_error(s.str());
    }
    return period_[0];
  }

  double phase() const {
    if (!is_const_phase_) {
      std::stringstream s;
      s << "The phase vector, [" << phase_ << "], cannot be represented as a "
        << "scalar value. Please use "
        << "drake::systems::Square::phase_vector() instead.";
      throw std::logic_error(s.str().c_str());
    }
    return phase_[0];
  }

  bool is_time_based() const { return is_time_based_; }

  const Eigen::VectorXd& amplitude_vector() const { return amplitude_; }

  const Eigen::VectorXd& pulse_width_vector() const { return pulse_width_; }

  const Eigen::VectorXd& period_vector() const { return period_; }

  const Eigen::VectorXd& phase_vector() const { return phase_; }

 private:
  void CalcValueOutput(const Context<double>& context,
                       BasicVector<double>* output) const {
    Eigen::VectorBlock<drake::VectorX<double>> output_block =
        output->get_mutable_value();

    const double time = context.get_time();

    for (int i = 0; i < pulse_width_.size(); ++i) {
      double t = time + phase_[i];
      if (!is_time_based_) {
        t = this->get_input_port(0).Eval(context)[i] + phase_[i];
      }
      output_block[i] =
          amplitude_[i] *
          (t - floor(t / period_[i]) * period_[i] < pulse_width_[i] ? 1 : 0);
    }
  }

  const Eigen::VectorXd amplitude_;
  const Eigen::VectorXd pulse_width_;
  const Eigen::VectorXd period_;
  const Eigen::VectorXd phase_;
  const bool is_time_based_;
  bool is_const_amplitude_{false};
  bool is_const_pulse_width_{false};
  bool is_const_period_{false};
  bool is_const_phase_{false};

  int value_output_port_index_{-1};
};

int DoMain() {
  // Construct a MultibodyPlant and a SceneGraph.
  drake::systems::DiagramBuilder<double> builder;

  MultibodyPlantConfig plant_config;
  plant_config.time_step = FLAGS_mbp_discrete_update_period;
  plant_config.stiction_tolerance = FLAGS_stiction_tolerance;
  plant_config.contact_model = FLAGS_contact_model;
  plant_config.contact_surface_representation =
      FLAGS_contact_surface_representation;

  MultibodyPlant<double>* plant;
  SceneGraph<double>* scene_graph;

  DRAKE_DEMAND(FLAGS_mbp_discrete_update_period >= 0);
  std::tie(plant, scene_graph) =
      multibody::AddMultibodyPlant(plant_config, &builder);

  // Parse the gripper and spatula models.
  drake::multibody::Parser parser(plant, scene_graph);
  const std::string gripper_file = FindResourceOrThrow(
      "drake/manipulation/models/wsg_50_hydro_bubble_description/sdf/"
      "schunk_wsg_50_hydro_bubble.sdf");
  const std::string spatula_file = FindResourceOrThrow(
      "drake/examples/multibody/spatula_slip_control/models/spatula.sdf");
  parser.AddModelFromFile(gripper_file);
  auto spatula_instance = parser.AddModelFromFile(spatula_file);
  // Pose the gripper and weld it to the world.
  const drake::math::RigidTransform<double> X_WF0 =
      drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw(0.0, -1.57, 0.0),
          Eigen::Vector3d(0, 0, 0.25));
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("gripper"),
                    X_WF0);
  plant->Finalize();

  // Construct the open loop square wave controller. To oscillate around a
  // constant force, we construct a ConstantVectorSource and combine it with
  // the square wave output using an Adder.
  const double f0 = FLAGS_gripper_force;

  const Eigen::Vector2d amplitudes(FLAGS_amplitude, -FLAGS_amplitude);
  const Eigen::Vector2d pulse_widths(FLAGS_pulse_width, FLAGS_pulse_width);
  const Eigen::Vector2d periods(FLAGS_period, FLAGS_period);
  const Eigen::Vector2d phases(0, 0);
  const auto& square_force =
      *builder.AddSystem<Square>(amplitudes, pulse_widths, periods, phases);
  const auto& constant_force =
      *builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
          Eigen::Vector2d(f0, -f0));
  const auto& adder = *builder.AddSystem<drake::systems::Adder<double>>(2, 2);
  builder.Connect(square_force.get_output_port(0), adder.get_input_port(0));
  builder.Connect(constant_force.get_output_port(), adder.get_input_port(1));

  // Connect the output of the adder to the plant's actuation input.
  builder.Connect(adder.get_output_port(0), plant->get_actuation_input_port());

  // Create a visualizer for the system and ensure contact results are
  // visualized.
  drake::geometry::DrakeVisualizerParams params;
  params.role = FLAGS_visualize_collision
                    ? drake::geometry::Role::kProximity
                    : drake::geometry::Role::kIllustration;
  drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, *scene_graph,
                                                  /* lcm */ nullptr, params);
  drake::multibody::ConnectContactResultsToDrakeVisualizer(&builder, *plant,
                                                           *scene_graph,
                                                           /* lcm */ nullptr);

  // Construct a simulator.
  std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();

  SimulatorConfig sim_config;
  sim_config.integration_scheme = FLAGS_integration_scheme;
  sim_config.max_step_size = FLAGS_max_time_step;
  sim_config.accuracy = FLAGS_accuracy;
  sim_config.target_realtime_rate = FLAGS_realtime_rate;
  sim_config.publish_every_time_step = false;

  std::unique_ptr<drake::systems::Simulator<double>> simulator =
      std::make_unique<drake::systems::Simulator<double>>(*diagram);
  ApplySimulatorConfig(simulator.get(), sim_config);

  // Set the initial conditions for the spatula pose and the gripper finger
  // positions.
  Context<double>& mutable_root_context = simulator->get_mutable_context();
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(*plant, &mutable_root_context);

  // Set spatulate free body pose.
  const drake::math::RigidTransform<double> X_WF1 =
      drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw(-0.4, 0.0, 1.57),
          Eigen::Vector3d(0.35, 0, 0.25));
  const auto& base_link = plant->GetBodyByName("spatula", spatula_instance);
  plant->SetFreeBodyPose(&plant_context, base_link, X_WF1);

  // Set finger joint positions.
  const PrismaticJoint<double>& left_joint =
      plant->GetJointByName<PrismaticJoint>("left_finger_sliding_joint");
  left_joint.set_translation(&plant_context, -0.01);
  const PrismaticJoint<double>& right_joint =
      plant->GetJointByName<PrismaticJoint>("right_finger_sliding_joint");
  right_joint.set_translation(&plant_context, 0.01);

  // Simulate.
  simulator->Initialize();
  simulator->AdvanceTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace spatula_slip_control
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
    gflags::SetUsageMessage(
      "This is an example of using the hydroelastic contact model with a\n"
      "robot gripper with compliant bubble fingers and a compliant spatula.\n"
      "The example poses the spatula in the closed grip of the gripper and\n"
      "uses an open loop square wave controller to perform a controlled\n"
      "rotational slip of the spatula while maintaining the spatula in\n"
      "the gripper's grasp. Launch drake-visualizer before running this\n"
      "example. See the README.md file for more information.\n");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::spatula_slip_control::DoMain();
}
