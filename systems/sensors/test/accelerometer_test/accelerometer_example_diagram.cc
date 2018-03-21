#include "drake/systems/sensors/test/accelerometer_test/accelerometer_example_diagram.h"

#include <memory>
#include <string>
#include <utility>

#include "drake/common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"

using std::make_unique;
using std::move;
using std::unique_ptr;

namespace drake {

using parsers::ModelInstanceIdTable;
using parsers::urdf::AddModelInstanceFromUrdfFileToWorld;

namespace systems {
namespace sensors {

AccelerometerExampleDiagram::AccelerometerExampleDiagram() {
  const std::string model_file_name =
      FindResourceOrThrow("drake/examples/pendulum/Pendulum.urdf");
  const std::string model_name = "Pendulum";

  auto tree = make_unique<RigidBodyTree<double>>();
  tree_ = tree.get();

  // Adds a pendulum to the RigidBodyTree and obtains its model instance ID.
  const parsers::ModelInstanceIdTable table =
      AddModelInstanceFromUrdfFileToWorld(
          model_file_name, drake::multibody::joints::kFixed, tree_);
  model_instance_id_ = table.at(model_name);

  // Specifies the location of the accelerometer sensor.
  Eigen::Isometry3d sensor_frame_transform = Eigen::Isometry3d::Identity();
  sensor_frame_transform.translation() << 0, 0, -0.5;

  // Adds a frame to the RigidBodyTree called "sensor frame" that is coincident
  // with the "arm" body within the RigidBodyTree.
  sensor_frame_ = std::allocate_shared<RigidBodyFrame<double>>(
      Eigen::aligned_allocator<RigidBodyFrame<double>>(), "sensor frame",
      tree->FindBody("arm"), sensor_frame_transform);
  tree->addFrame(sensor_frame_);

  plant_ = builder_.template AddSystem<RigidBodyPlant<double>>(move(tree));
  plant_->set_name("plant");

  accelerometer_ = Accelerometer::AttachAccelerometer(
      "my accelerometer", *sensor_frame_, *plant_, true /* include_gravity */,
      &builder_);

  logger_ = builder_.template AddSystem<AccelerometerTestLogger>(
      plant_->get_num_states());
  logger_->set_name("logger");

  auto constant_zero_source =
      builder_.template AddSystem<ConstantVectorSource<double>>(
          VectorX<double>::Zero(plant_->actuator_command_input_port().size()));
  constant_zero_source->set_name("zero");

  builder_.Connect(constant_zero_source->get_output_port(),
                   plant_->actuator_command_input_port());
  builder_.Connect(plant_->state_output_port(),
                   logger_->get_plant_state_input_port());
  builder_.Connect(plant_->state_derivative_output_port(),
                   accelerometer_->get_plant_state_derivative_input_port());
  builder_.Connect(plant_->state_derivative_output_port(),
                   logger_->get_plant_state_derivative_input_port());
  builder_.Connect(accelerometer_->get_output_port(),
                   logger_->get_acceleration_input_port());
}

void AccelerometerExampleDiagram::Initialize(
    unique_ptr<DrakeVisualizer> visualizer) {
  if (visualizer != nullptr) {
    visualizer_ = builder_.AddSystem(move(visualizer));
    if (visualizer_->get_name().empty()) {
      visualizer_->set_name("visualizer");
    }
    builder_.Connect(plant_->state_output_port(),
                     visualizer_->get_input_port(0));
  }

  builder_.BuildInto(this);
}

void AccelerometerExampleDiagram::SetInitialState(Context<double>* context,
    double q, double v) {
  Context<double>& plant_context =
      GetMutableSubsystemContext(*plant_, context);
  ContinuousState<double>& plant_state =
      plant_context.get_mutable_continuous_state();
  DRAKE_DEMAND(plant_state.size() == 2);
  plant_state[0] = q;
  plant_state[1] = v;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
