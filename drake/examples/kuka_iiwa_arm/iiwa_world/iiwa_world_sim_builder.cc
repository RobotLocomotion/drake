#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_sim_builder.h"

#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/framework/primitives/constant_vector_source.h"
#include "drake/systems/framework/primitives/multiplexer.h"

using Eigen::aligned_allocator;
using Eigen::Vector3d;
using drake::lcm::DrakeLcm;
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kQuaternion;
using drake::parsers::ModelInstanceIdTable;
using drake::parsers::sdf::AddModelInstancesFromSdfFile;
using drake::parsers::urdf::AddModelInstanceFromUrdfFile;
using drake::systems::ConstantVectorSource;
using drake::systems::Context;
using drake::systems::ContinuousState;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::DrakeVisualizer;
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::systems::VectorBase;
using std::allocate_shared;
using std::make_unique;
using std::string;
using std::unique_ptr;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
IiwaWorldSimBuilder<T>::IiwaWorldSimBuilder() {}

template <typename T>
IiwaWorldSimBuilder<T>::~IiwaWorldSimBuilder() {}

template <typename T>
int IiwaWorldSimBuilder<T>::AddFixedModelInstance(const string& model_name,
                                                  const Vector3d& xyz,
                                                  const Vector3d& rpy) {
  DRAKE_DEMAND(!built_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), "world", nullptr, xyz, rpy);

  return AddModelInstanceToFrame(model_name, xyz, rpy, weld_to_frame);
}

template <typename T>
int IiwaWorldSimBuilder<T>::AddFloatingModelInstance(const string& model_name,
                                                     const Vector3d& xyz,
                                                     const Vector3d& rpy) {
  DRAKE_DEMAND(!built_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), "world", nullptr, xyz, rpy);

  return AddModelInstanceToFrame(model_name, xyz, rpy, weld_to_frame,
                                 kQuaternion);
}

template <typename T>
int IiwaWorldSimBuilder<T>::AddModelInstanceToFrame(
    const string& model_name, const Vector3d& xyz, const Vector3d& rpy,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    const drake::multibody::joints::FloatingBaseType floating_base_type) {
  DRAKE_DEMAND(!built_);
  std::size_t extension_location = model_map_[model_name].find_last_of(".");

  DRAKE_DEMAND(extension_location < model_map_[model_name].size());

  std::string extension = model_map_[model_name].substr(
      extension_location + 1);

  parsers::ModelInstanceIdTable table;

  DRAKE_DEMAND(extension == "urdf" || extension == "sdf");

  if (extension == "urdf") {
    table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        drake::GetDrakePath() + model_map_[model_name], floating_base_type,
        weld_to_frame, rigid_body_tree_.get());

  } else if (extension == "sdf") {
    table = drake::parsers::sdf::AddModelInstancesFromSdfFile(
        drake::GetDrakePath() + model_map_[model_name], floating_base_type,
        weld_to_frame, rigid_body_tree_.get());
  }
  const int model_instance_id = table.begin()->second;
  return model_instance_id;
}

template <typename T>
void IiwaWorldSimBuilder<T>::AddGround() {
  DRAKE_DEMAND(!built_);
  drake::multibody::AddFlatTerrainToWorld(rigid_body_tree_.get());
}

template <typename T>
std::unique_ptr<systems::Diagram<T>> IiwaWorldSimBuilder<T>::Build() {
  DRAKE_DEMAND(!built_);

  std::unique_ptr<systems::DiagramBuilder<T>> builder{
      make_unique<systems::DiagramBuilder<T>>()};

  plant_ = builder->template AddSystem<systems::RigidBodyPlant<T>>(
      move(rigid_body_tree_));

  DRAKE_DEMAND(plant_ != nullptr);
  plant_->set_contact_parameters(penetration_stiffness_, penetration_damping_,
                                 contact_friction_);

  DRAKE_DEMAND(plant_->get_num_actuators() > 0);

  // Creates and adds a DrakeVisualizer publisher.
  auto viz_publisher_ = builder->template AddSystem<DrakeVisualizer>(
      plant_->get_rigid_body_tree(), &lcm_);

  // Connects the plant to the publisher for visualization.
  builder->Connect(plant_->get_output_port(0),
                   viz_publisher_->get_input_port(0));

  // Exposes output and input ports of the Diagram.
  builder->ExportOutput(plant_->get_output_port(0));
  builder->ExportInput(plant_->get_input_port(0));

  drake::log()->debug("Plant Diagram initialized...");
  built_ = true;

  return std::move(builder->Build());
}

template <typename T>
void IiwaWorldSimBuilder<T>::SetZeroConfiguration(
    systems::Simulator<T>* simulator,
    const systems::Diagram<T>* demo_diagram) {
  DRAKE_DEMAND(simulator != nullptr && demo_diagram != nullptr);

  std::vector<const systems::System<double>* > demo_systems =
      demo_diagram->GetSystems();

  const systems::Diagram<T>* plant_and_visualizer_diagram =
      dynamic_cast<const systems::Diagram<T>*>(demo_systems.at(0));
  DRAKE_DEMAND(plant_and_visualizer_diagram != nullptr);

  std::vector<const systems::System<double>*> plant_and_visualizer_subsystems =
      plant_and_visualizer_diagram->GetSystems();

  const RigidBodyPlant<double>* rigid_body_plant =
      dynamic_cast<const RigidBodyPlant<double>*>(
          plant_and_visualizer_subsystems.at(0));

  DRAKE_DEMAND(rigid_body_plant != nullptr);

  Context<double>* input_diagram_context =
      demo_diagram->GetMutableSubsystemContext(
          simulator->get_mutable_context(), plant_and_visualizer_diagram);
  DRAKE_DEMAND(plant_and_visualizer_diagram != nullptr);

  Context<double>* plant_context =
      plant_and_visualizer_diagram->GetMutableSubsystemContext(
          input_diagram_context, rigid_body_plant);

  rigid_body_plant->SetZeroConfiguration(plant_context);
}

template <typename T>
void IiwaWorldSimBuilder<T>::SetPenetrationContactParameters(
    double penetration_stiffness, double penetration_damping,
    double contact_friction) {
  DRAKE_DEMAND(!built_);
  penetration_stiffness_ = penetration_stiffness;
  penetration_damping_ = penetration_damping;
  contact_friction_ = contact_friction;
}

template <typename T>
void IiwaWorldSimBuilder<T>::StoreModel(const std::string& model_name,
                                        const std::string& model_path) {
  model_map_.insert(
      std::pair<std::string, std::string>(model_name, model_path));
}

template <typename T>
int IiwaWorldSimBuilder<T>::GetPlantInputSize() {
  return plant_->get_input_size();
}

template class IiwaWorldSimBuilder<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
