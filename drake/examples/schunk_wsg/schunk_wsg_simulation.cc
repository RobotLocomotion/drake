/// @file
///
/// Implements a simulation of the Schunk WSG 50 gripper.  Like the
/// driver for the physical gripper, this simulation communicates over
/// LCM using the lcmt_schunk_status and lcmt_schunk_command messages,
/// It is intended to be a direct replacement for the schunk driver.

#include <cmath>
#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/trajectories/piecewise_polynomial_trajectory.h"
#include "drake/examples/schunk_wsg/simulated_schunk_wsg_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/controllers/pid_controlled_system.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/matrix_gain.h"
#include "drake/systems/framework/primitives/multiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/lcmt_schunk_wsg_command.hpp"
#include "drake/lcmt_schunk_wsg_status.hpp"

DEFINE_double(simulation_sec, std::numeric_limits<double>::infinity(),
              "Number of seconds to simulate.");

namespace drake {
namespace examples {
namespace schunk_wsg {
namespace {

using systems::ConstantVectorSource;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::MatrixGain;
using systems::Multiplexer;
using systems::PidControlledSystem;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::SystemOutput;

// TODO(sam.creasey) Right now this class just outputs a position to
// control to, which is not going to be sufficient to capture the
// entire control state of the gripper (particularly the maximum
// force).
class SchunkWsgCommandReceiver : public systems::LeafSystem<double> {
 public:
  explicit SchunkWsgCommandReceiver(int position_idx)
      : position_idx_(position_idx) {
    this->set_name("SchunkWsgCommandReceiver");
    this->DeclareAbstractInputPort(systems::kContinuousSampling);
    this->DeclareInputPort(systems::kVectorValued, 10,
                           systems::kContinuousSampling);
    this->DeclareOutputPort(systems::kVectorValued, 2,
                            systems::kContinuousSampling);
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const {
    const systems::AbstractValue* input = this->EvalAbstractInput(context, 0);
    DRAKE_ASSERT(input != nullptr);
    const auto& command = input->GetValue<lcmt_schunk_wsg_command>();
    double target_position = -(command.target_position_mm / 1e3) / 2.;
    if (std::isnan(target_position)) {
      target_position = 0;
    }

    // Based on manually driving the actual gripper using the web
    // interface, it appears that it will at least attempt to respond
    // to commands as small as 0.1mm.
    const double kTargetEpsilon = 0.0001;

    const systems::BasicVector<double>* state =
          this->EvalVectorInput(context, 1);
    const double cur_position = state->GetAtIndex(position_idx_);

    if (std::abs(last_target_position_ - target_position) > kTargetEpsilon) {
      UpdateTrajectory(cur_position, target_position);
      last_target_position_ = target_position;
      trajectory_start_time_ = context.get_time();
    }

    if (trajectory_) {
      this->GetMutableOutputVector(output, 0) =
          trajectory_->value(context.get_time() - trajectory_start_time_);
    } else {
      this->GetMutableOutputVector(output, 0) =
          Eigen::Vector2d(cur_position, 0);
    }
  }

 private:
  void UpdateTrajectory(double cur_position, double target_position) const {
    // The acceleration and velocity limits correspond to the maximum
    // values available for manual control through the gripper's web
    // interface.
    const double kMaxVelocity = 0.42;  // m/s
    const double kMaxAccel = 5.;  // m/s^2
    const double kTimeToMaxAccel = kMaxVelocity / kMaxAccel;
    // TODO(sam.creasey) this should probably consider current speed
    // if the gripper is already moving.
    const double kDistanceToMaxAccel =
        0.5 * kMaxAccel * kTimeToMaxAccel * kTimeToMaxAccel;

    std::vector<Eigen::MatrixXd> knots;
    std::vector<double> times;
    knots.push_back(Eigen::Vector2d(cur_position, 0));
    times.push_back(0);

    const double direction = (cur_position < target_position) ? 1  : -1;
    const double delta = std::abs(target_position - cur_position);

    // The trajectory creation code below is, to say the best, a bit
    // primitive.  I (sam.creasey) would not be surprised if it could
    // be significantly improved.  It's also based only on the
    // configurable constants for the WSG 50, not on analysis of the
    // actual motion of the gripper.
    if (delta < kDistanceToMaxAccel * 2) {
      const double mid_distance = delta / 2;
      const double mid_velocity =
          kMaxVelocity * (mid_distance / kDistanceToMaxAccel);
      const double mid_time = mid_velocity / kMaxAccel;
      knots.push_back(Eigen::Vector2d(cur_position + mid_distance * direction,
                                      mid_velocity * direction));
      times.push_back(mid_time);
      knots.push_back(Eigen::Vector2d(target_position, 0));
      times.push_back(mid_time * 2);
    } else {
      knots.push_back(Eigen::Vector2d(
          cur_position + (kDistanceToMaxAccel * direction),
          kMaxVelocity * direction));
      times.push_back(kTimeToMaxAccel);

      const double time_at_max =
          (delta - 2 * kDistanceToMaxAccel) / kMaxVelocity;
      knots.push_back(Eigen::Vector2d(
          target_position - (kDistanceToMaxAccel * direction),
          kMaxVelocity * direction));
      times.push_back(kTimeToMaxAccel + time_at_max);

      knots.push_back(Eigen::Vector2d(target_position, 0));
      times.push_back(kTimeToMaxAccel + time_at_max + kTimeToMaxAccel);
    }
    trajectory_.reset(new PiecewisePolynomialTrajectory(
        PiecewisePolynomial<double>::FirstOrderHold(times, knots)));
  }

  int position_idx_;
  mutable double last_target_position_{};
  mutable double trajectory_start_time_{};
  mutable std::unique_ptr<Trajectory> trajectory_;
};

// This system has one input port for the current state of the plant.
class SchunkWsgStatusSender : public systems::LeafSystem<double> {
 public:
  SchunkWsgStatusSender(int position_idx, int velocity_idx)
      : position_idx_(position_idx), velocity_idx_(velocity_idx) {
    this->set_name("SchunkWsgStatusSender");
    this->DeclareInputPort(systems::kVectorValued, 10,
                           systems::kContinuousSampling);
    this->DeclareAbstractOutputPort(systems::kContinuousSampling);
  }

  std::unique_ptr<SystemOutput<double>> AllocateOutput(
      const Context<double>& context) const override {
    auto output = std::make_unique<systems::LeafSystemOutput<double>>();
    lcmt_schunk_wsg_status msg{};
    output->get_mutable_ports()->emplace_back(
        std::make_unique<systems::OutputPort>(
            std::make_unique<systems::Value<lcmt_schunk_wsg_status>>(msg)));
    return std::unique_ptr<SystemOutput<double>>(output.release());
  }

  void EvalOutput(const Context<double>& context,
                  SystemOutput<double>* output) const override {
    systems::AbstractValue* mutable_data = output->GetMutableData(0);
    lcmt_schunk_wsg_status& status =
        mutable_data->GetMutableValue<lcmt_schunk_wsg_status>();

    status.utime = context.get_time() * 1e6;
    const systems::BasicVector<double>* state =
        this->EvalVectorInput(context, 0);
    status.actual_position_mm = -2 * state->GetAtIndex(position_idx_) * 1e3;
    // TODO(sam.creasey) Figure out how to get the actual force from
    // the plant so that we can populate this field.
    status.actual_force = 0;
    status.actual_speed_mm_per_s = -2 * state->GetAtIndex(velocity_idx_) * 1e3;
  }

 private:
  const int position_idx_{};
  const int velocity_idx_{};
};

template <typename T>
class PidControlledSchunkWsg : public systems::Diagram<T> {
 public:
  PidControlledSchunkWsg() {
    this->set_name("PidControlledSchunkWsg");

    DiagramBuilder<T> builder;

    std::unique_ptr<RigidBodyPlant<T>> plant =
        CreateSimulatedSchunkWsgSystem<T>();
    plant_ = plant.get();

    // Create a source to emit a single zero.  We'll need this to
    // express external commanded force to the PidControlledSystem.
    auto zero_source = builder.template AddSystem<ConstantVectorSource<T>>(
        Eigen::VectorXd::Zero(1));

    const RigidBodyTree<T>& tree = plant->get_rigid_body_tree();
    const std::map<std::string, int> index_map =
        tree.computePositionNameToIndexMap();
    const int left_finger_position_index =
        index_map.at("left_finger_sliding_joint");
    position_idx_ = left_finger_position_index;
    velocity_idx_ = left_finger_position_index +
        plant->get_num_positions();

    Eigen::MatrixXd feedback_matrix = Eigen::MatrixXd::Zero(
        2 * plant->get_num_actuators(),
        2 * plant->get_num_positions());
    feedback_matrix(0, position_idx_) = 1.;
    feedback_matrix(1, velocity_idx_) = 1.;
    std::unique_ptr<MatrixGain<T>> feedback_selector =
        std::make_unique<MatrixGain<T>>(feedback_matrix);

    // TODO(sam.creasey) The choice of constants below is completely
    // arbitrary and may not match the performance of the actual
    // gripper.
    const T kp = 20.0;
    const T ki = 0.0;
    const T kd = 5.0;
    controller_ = builder.template AddSystem<PidControlledSystem<T>>(
        std::move(plant), std::move(feedback_selector), kp, ki, kd);

    builder.Connect(zero_source->get_output_port(),
                    controller_->get_input_port(0));

    builder.ExportInput(controller_->get_input_port(1));
    builder.ExportOutput(controller_->get_output_port(0));
    builder.BuildInto(this);
  }

  void SetDefaultState(Context<T>* context) const {
    Context<T>* controller_context =
        this->GetMutableSubsystemContext(context, controller_);
    controller_->SetDefaultState(controller_context);

    Context<T>* plant_context =
        controller_->GetMutableSubsystemContext(controller_context, plant_);
    plant_->SetZeroConfiguration(plant_context);
  }

  const RigidBodyPlant<T>& get_plant() const { return *plant_; }
  int position_idx() const { return position_idx_; }
  int velocity_idx() const { return velocity_idx_; }

 private:
  RigidBodyPlant<T>* plant_{nullptr};
  PidControlledSystem<T>* controller_{nullptr};
  int position_idx_{};
  int velocity_idx_{};
};

int DoMain() {
  systems::DiagramBuilder<double> builder;
  auto model = builder.AddSystem<PidControlledSchunkWsg<double>>();

  const RigidBodyTree<double>& tree =
      model->get_plant().get_rigid_body_tree();

  // Creates and adds LCM publisher for visualization.
  drake::lcm::DrakeLcm lcm;
  DrakeVisualizer* visualizer =
      builder.AddSystem<DrakeVisualizer>(tree, &lcm);

  // Create the command subscriber and status publisher.
  auto command_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<lcmt_schunk_wsg_command>(
          "SCHUNK_WSG_COMMAND", &lcm));
  auto command_receiver =
      builder.AddSystem<SchunkWsgCommandReceiver>(model->position_idx());

  auto status_pub = builder.AddSystem(
      systems::lcm::LcmPublisherSystem::Make<lcmt_schunk_wsg_status>(
          "SCHUNK_WSG_STATUS", &lcm));
  auto status_sender = builder.AddSystem<SchunkWsgStatusSender>(
      model->position_idx(), model->velocity_idx());

  builder.Connect(command_sub->get_output_port(0),
                  command_receiver->get_input_port(0));
  builder.Connect(command_receiver->get_output_port(0),
                  model->get_input_port(0));
  builder.Connect(model->get_output_port(0),
                  visualizer->get_input_port(0));
  builder.Connect(model->get_output_port(0),
                  status_sender->get_input_port(0));
  builder.Connect(model->get_output_port(0),
                  command_receiver->get_input_port(1));
  builder.Connect(status_sender->get_output_port(0),
                  status_pub->get_input_port(0));
  auto sys = builder.Build();

  Simulator<double> simulator(*sys);

  // Zeroes the state and initializes controller state.
  auto model_context = sys->GetMutableSubsystemContext(
      simulator.get_mutable_context(), model);
  model->SetDefaultState(model_context);

  lcm.StartReceiveThread();
  simulator.Initialize();

  // Simulate for a very long time.
  simulator.StepTo(FLAGS_simulation_sec);
  return 0;
}

}  // namespace
}  // namespace schunk_wsg
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::examples::schunk_wsg::DoMain();
}
