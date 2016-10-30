#include <gflags/gflags.h>

#include "drake/common/drake_path.h"
#include "drake/common/text_logging_gflags.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/continuous_state.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/plants/parser_urdf.h"
#include "drake/systems/plants/parser_model_instance_id_table.h"
#include "drake/systems/plants/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/plants/rigid_body_plant/rigid_body_plant.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFile;

using lcm::DrakeLcm;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::VectorBase;
using systems::plants::joints::kFixed;

namespace examples {
namespace kuka_iiwa_arm {
namespace {

DEFINE_double(duration, 5, "Total duration of the simulation in seconds.");

// A demo of an uncontrolled KUKA iiwa arm.
template<typename T>
class KukaIiwaArmDynamicsSim : public systems::Diagram<T> {
 public:
  KukaIiwaArmDynamicsSim() {
    this->set_name("KukaIiwaArmDynamicsSim");

    // Instantiates an Multibody Dynamics (MBD) model of the world.
    auto tree = make_unique<RigidBodyTree>();
    ModelInstanceIdTable vehicle_instance_id_table =
        drake::parsers::urdf::AddModelInstanceFromUrdfFile(
            drake::GetDrakePath() + "/examples/kuka_iiwa_arm/urdf/iiwa14.urdf",
            kFixed, nullptr /* weld to frame */, tree.get());

    AddGround(tree.get());

    DiagramBuilder<T> builder;

    // Instantiates a RigidBodyPlant from the MBD model of the world.
    plant_ = builder.template AddSystem<RigidBodyPlant<T>>(move(tree));
    plant_->set_contact_parameters(3000 /* penetration stiffness */,
        0 /* penetration damping */, 1.0 /* friction coefficient */);

    // Feed in constant inputs of zero into the RigidBodyPlant.
    VectorX<T> constant_value(plant_->get_input_size());
    constant_value.setZero();
    const_source_ = builder.template AddSystem<ConstantVectorSource<T>>(
        constant_value);

    // Creates and adds LCM publisher for visualization.
    viz_publisher_ = builder.template AddSystem<DrakeVisualizer>(
        plant_->get_rigid_body_tree(), &lcm_);

    // Connects the constant source output port to the RigidBodyPlant's input
    // port. This effectively results in the robot being uncontrolled.
    builder.Connect(const_source_->get_output_port(),
                    plant_->get_input_port(0));

    // Connects to publisher for visualization.
    builder.Connect(plant_->get_output_port(0),
                    viz_publisher_->get_input_port(0));

    builder.ExportOutput(plant_->get_output_port(0));
    builder.BuildInto(this);
  }

  void SetDefaultState(Context<T>* context) const {
    Context<T>* plant_context =
        this->GetMutableSubsystemContext(context, plant_);
    plant_->SetZeroConfiguration(plant_context);
  }

  const RigidBodyPlant<T>& get_rigid_body_plant() {
    return *plant_;
  }

 private:
  RigidBodyPlant<T>* plant_;
  DrakeVisualizer* viz_publisher_;
  DrakeLcm lcm_;

  ConstantVectorSource<T>* const_source_;
};

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  logging::HandleSpdlogGflags();

  KukaIiwaArmDynamicsSim<double> model;
  Simulator<double> simulator(model);

  // Initializes the controller state and system state to be all zero.
  model.SetDefaultState(simulator.get_mutable_context());

  simulator.Initialize();

  // Simulate for the desired duration.
  simulator.StepTo(FLAGS_duration);

  // Ensures the simulation was successful.
  const RigidBodyPlant<double>& rigid_body_plant = model.get_rigid_body_plant();
  const Context<double>& context = simulator.get_context();
  const ContinuousState<double>* state = context.get_continuous_state();
  const VectorBase<double>& position_vector = state->get_generalized_position();
  const VectorBase<double>& velocity_vector = state->get_generalized_velocity();

  const int num_q = position_vector.size();
  const int num_v = velocity_vector.size();

  // Ensures the sizes of the position and velocity vectors are correct.
  if (num_q != rigid_body_plant.get_num_positions()) {
    throw std::runtime_error(
        "ERROR: Size of position vector (" + std::to_string(num_q) + ") does "
        "not match number of positions in RigidBodyTree (" +
        std::to_string(rigid_body_plant.get_num_positions()) + ").");
  }

  if (num_v != rigid_body_plant.get_num_velocities()) {
    throw std::runtime_error(
        "ERROR: Size of velocity vector (" + std::to_string(num_v) + ") does "
        "not match number of velocities in RigidBodyTree (" +
        std::to_string(rigid_body_plant.get_num_velocities()) + ").");
  }

  // Ensures the number of position states equals the number of velocity states.
  if (num_q != num_v) {
    throw std::runtime_error(
        "ERROR: Number of positions (" + std::to_string(num_q) + ") does not "
        "match the number of velocities (" + std::to_string(num_v) + ").");
  }

  // Ensures the robot's joints are within their position limits.
  const std::vector<std::unique_ptr<RigidBody>>& bodies =
      rigid_body_plant.get_rigid_body_tree().bodies;
  for (int state_index = 0, i = 0; i < static_cast<int>(bodies.size()); ++i) {
    // Skips rigid bodies without a parent. This includes the world.
    if (!bodies[i]->has_parent_body()) continue;

    const DrakeJoint& joint = bodies[i]->getJoint();
    const Eigen::VectorXd& min_limit = joint.getJointLimitMin();
    const Eigen::VectorXd& max_limit = joint.getJointLimitMax();

    // Defines a joint limit tolerance. This is the amount in radians over which
    // joint position limits can be violated and still be considered to be
    // within the limits. Once we are able to model joint limits via
    // constraints, we may be able to remove this tolerance value.
    const double kJointLimitTolerance = 0.0261799;  // 1.5 degrees.

    for (int j = 0; j < joint.get_num_positions(); ++j) {
      double position = position_vector.GetAtIndex(state_index++);
      if (position < min_limit[j] - kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(j) +
                                 ") violated minimum position limit (" +
                                 std::to_string(position) + " < " +
                                 std::to_string(min_limit[j]) + ").");
      }
      if (position > max_limit[j] + kJointLimitTolerance) {
        throw std::runtime_error("ERROR: Joint " + joint.get_name() + " (DOF " +
                                 joint.get_position_name(j) +
                                 ") violated maximum position limit (" +
                                 std::to_string(position) + " > " +
                                 std::to_string(max_limit[j]) + ").");
      }
    }
  }

  return 0;
}

}  // namespace
}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::examples::kuka_iiwa_arm::main(argc, argv);
}
