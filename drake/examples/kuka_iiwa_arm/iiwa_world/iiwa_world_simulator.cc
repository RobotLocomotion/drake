#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_simulator.h"

#include "drake/common/drake_export.h"
#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/RigidBodyFrame.h"
#include "drake/multibody/RigidBodyTree.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"

using drake::multibody::joints::kFixed;
using drake::multibody::joints::kQuaternion;
using drake::multibody::joints::FloatingBaseType;
using drake::lcm::DrakeLcm;
using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::ContinuousState;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::DrakeVisualizer;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::systems::VectorBase;
using drake::parsers::ModelInstanceIdTable;
using drake::parsers::sdf::AddModelInstancesFromSdfFile;
using drake::parsers::urdf::AddModelInstanceFromUrdfFile;
using Eigen::aligned_allocator;
using Eigen::Vector3d;
using std::allocate_shared;
using std::string;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaWorldSimulator<T>::IiwaWorldSimulator() {}

template <typename T>
IiwaWorldSimulator<T>::~IiwaWorldSimulator() {}

template <typename T>
int IiwaWorldSimulator<T>::AddObjectFixedToWorld(Vector3d xyz, Vector3d rpy,
                                                 string object_name) {
  DRAKE_DEMAND(!started_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), "world", nullptr, xyz, rpy);

  return AddObjectToFrame(xyz, rpy, object_name, weld_to_frame);
}

template <typename T>
int IiwaWorldSimulator<T>::AddObjectFloatingToWorld(Vector3d xyz, Vector3d rpy,
                                                    string object_name) {
  DRAKE_DEMAND(!started_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), "world", nullptr, xyz, rpy);

  return AddObjectToFrame(xyz, rpy, object_name, weld_to_frame, kQuaternion);
}

template <typename T>
int IiwaWorldSimulator<T>::AddObjectToFrame(
    Vector3d xyz, Vector3d rpy, string object_name,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    const drake::multibody::joints::FloatingBaseType floating_base_type) {
  DRAKE_DEMAND(!started_);
  std::size_t extension_location =
      object_name_map_[object_name].find_last_of(".");
  if (extension_location >= object_name_map_[object_name].size()) {
    return -1;
  }
  std::string extension =
      object_name_map_[object_name].substr(extension_location + 1);

  parsers::ModelInstanceIdTable table;
  if (extension.compare("urdf") == 0) {
    table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + object_name_map_[object_name],
        floating_base_type, weld_to_frame, rigid_body_tree_.get());

  } else if (extension.compare("sdf") == 0) {
    table = drake::parsers::sdf::AddModelInstancesFromSdfFile(
        drake::GetDrakePath() + object_name_map_[object_name],
        floating_base_type, weld_to_frame, rigid_body_tree_.get());
  } else {
    return -1;
  }

  const int model_instance_id = table.begin()->second;
  return model_instance_id;
}

template <typename T>
void IiwaWorldSimulator<T>::AddGroundToTree() {
  drake::multibody::AddFlatTerrainToWorld(rigid_body_tree_.get());
}

template <typename T>
void IiwaWorldSimulator<T>::Build() {
  DRAKE_DEMAND(!started_);

  auto plant = builder_->template AddSystem<systems::RigidBodyPlant<T>>(
      move(rigid_body_tree_));
  plant->set_contact_parameters(penetration_stiffness_, penetration_damping_,
                                contact_friction_);

  // Feed in constant inputs of zero into the RigidBodyPlant.
  VectorX<T> constant_value(plant->get_input_size());
  constant_value.setZero();

  auto const_source_ =
      builder_->template AddSystem<ConstantVectorSource<T>>(constant_value);

  // Creates and adds LCM publisher for visualization.
  auto viz_publisher_ = builder_->template AddSystem<DrakeVisualizer>(
      plant->get_rigid_body_tree(), &lcm_);

  // Connects the constant source output port to the RigidBodyPlant's input
  // port. This effectively results in the robot being uncontrolled.
  builder_->Connect(const_source_->get_output_port(), plant->get_input_port(0));

  // Connects to publisher for visualization.
  builder_->Connect(plant->get_output_port(0),
                    viz_publisher_->get_input_port(0));

  builder_->ExportOutput(plant->get_output_port(0));

  diagram_ = builder_->Build();

  simulator_ = std::make_unique<systems::Simulator<T>>(*diagram_);
  Context<T>* plant_context = diagram_->GetMutableSubsystemContext(
      simulator_->get_mutable_context(), plant);
  plant->SetZeroConfiguration(plant_context);

  simulator_->Initialize();

  std::cout << "Simulator initialised\n";

  started_ = true;
}

template <typename T>
void IiwaWorldSimulator<T>::StepBy(const T& time_step) {
  const T time = simulator_->get_context().get_time();
  SPDLOG_TRACE(drake::log(), "Time is now {}", time);
  simulator_->StepTo(time + time_step);
}

template <typename T>
void IiwaWorldSimulator<T>::StepTo(const T& final_time) {
  simulator_->StepTo(final_time);
}


template <typename T>
void IiwaWorldSimulator<T>::SetPenetrationContactParameters(
    double penetration_stiffness, double penetration_damping,
    double contact_friction) {
  penetration_stiffness_ = penetration_stiffness;
  penetration_damping_ = penetration_damping;
  contact_friction_ = contact_friction;
}

template class DRAKE_EXPORT IiwaWorldSimulator<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
