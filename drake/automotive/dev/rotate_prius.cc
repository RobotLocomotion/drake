/// @file
///
/// A simulation of a Prius attached to the world via a
/// RollPitchYawFloatingJoint. The floating joint is kept in its zero
/// position except for its yaw DOF, whose position is linearly increased over
/// time.
///
/// To run this simulation, execute:
///
/// <pre>
/// $ ./build/install/bin/drake-visualizer &
/// $ bazel run //drake/automotive:rotate_prius -- --rotation_rate=100
/// </pre>
///
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include "drake/automotive/gen/euler_floating_joint_state.h"
#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw_not_using_quaternion.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

DEFINE_double(rotation_rate, 10.0,
              "The rate at which the Prius model is being rotated in degrees "
              "per second.");

DEFINE_string(model,
    drake::GetDrakePath() +
      "/automotive/models/prius/prius_with_lidar.sdf",
      "The model to load into the simulator.");

using Eigen::Vector3d;

using std::numeric_limits;
using std::vector;

namespace drake {

using systems::BasicVector;
using systems::ConstantVectorSource;
using systems::Context;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::LeafSystem;
using systems::Multiplexer;
using systems::OutputPortDescriptor;
using systems::Simulator;
using systems::SystemOutput;

namespace automotive {
namespace {

class PriusPosePublisher : public LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PriusPosePublisher)

  PriusPosePublisher() {
    this->set_name("PriusPosePublisher");
    port_index_ =
        this->DeclareOutputPort(systems::kVectorValued,
            EulerFloatingJointStateIndices::kNumCoordinates).get_index();
  }

  const OutputPortDescriptor<double>& output_port() const {
    return get_output_port(port_index_);
  }

  void DoCalcOutput(const Context<double>& context,
                    SystemOutput<double>* output) const override {
    BasicVector<double>* const output_vector =
        output->GetMutableVectorData(port_index_);
    DRAKE_ASSERT(output_vector != nullptr);
    EulerFloatingJointState<double>* const output_data =
        dynamic_cast<EulerFloatingJointState<double>*>(output_vector);
    DRAKE_ASSERT(output_data != nullptr);

    const double yaw =
        FLAGS_rotation_rate / 180.0 * M_PI * context.get_time();

    // The following offsets the visualization model such that the root is below
    // the middle of the vehicle's rear axle.
    output_data->set_x(1.40948 * std::cos(yaw));
    output_data->set_y(1.40948 * std::sin(yaw));
    output_data->set_z(0);

    output_data->set_roll(0.0);
    output_data->set_pitch(0.0);
    output_data->set_yaw(yaw);
  }

 protected:
  std::unique_ptr<BasicVector<double>> AllocateOutputVector(
      const OutputPortDescriptor<double>& descriptor) const override {
    return std::make_unique<EulerFloatingJointState<double>>();
  }

 private:
  int port_index_{};
};

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::sdf::AddModelInstancesFromSdfFileToWorld(
      FLAGS_model, multibody::joints::kRollPitchYaw, tree.get());

  lcm::DrakeLcm real_lcm;

  DiagramBuilder<double> builder;
  auto visualizer =
      builder.template AddSystem<DrakeVisualizer>(*tree, &real_lcm);
  auto pose_publisher =
      builder.template AddSystem<PriusPosePublisher>();

  const int num_states = tree->get_num_positions() + tree->get_num_velocities();
  const int num_pose_states = pose_publisher->output_port().size();
  DRAKE_DEMAND(num_states >= num_pose_states);

  auto multiplexer =
      builder.template AddSystem<Multiplexer<double>>(
        vector<int>{num_pose_states, num_states - num_pose_states});

  auto zero_source =
      builder.template AddSystem<ConstantVectorSource>(
          VectorX<double>::Zero(num_states - num_pose_states).eval());

  builder.Connect(pose_publisher->output_port(),
                  multiplexer->get_input_port(0));
  builder.Connect(zero_source->get_output_port(),
                  multiplexer->get_input_port(1));
  builder.Connect(multiplexer->get_output_port(0),
                  visualizer->get_input_port(0));

  auto diagram = builder.Build();
  auto simulator = std::make_unique<Simulator<double>>(*diagram);
  real_lcm.StartReceiveThread();

  simulator->set_target_realtime_rate(1.0 /* target_realtime_rate */);
  simulator->get_mutable_integrator()->set_maximum_step_size(0.01);
  simulator->get_mutable_integrator()->set_minimum_step_size(0.01);
  simulator->Initialize();
  simulator->StepTo(numeric_limits<double>::max());

  return 0;
}

}  // namespace
}  // namespace automotive
}  // namespace drake

int main(int argc, char* argv[]) {return drake::automotive::main(argc, argv); }
