#include "drake/examples/kuka_iiwa_arm/iiwa_world/iiwa_world_sim_builder.h"

#include <utility>
#include <vector>

#include "drake/common/drake_path.h"
#include "drake/examples/examples_package_map.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_common.h"
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

using std::allocate_shared;
using std::make_unique;
using std::string;
using std::unique_ptr;

namespace drake {

using lcm::DrakeLcm;
using multibody::AddFlatTerrainToWorld;
using multibody::joints::FloatingBaseType;
using multibody::joints::kFixed;
using multibody::joints::kQuaternion;
using parsers::ModelInstanceIdTable;
using parsers::PackageMap;
using parsers::sdf::AddModelInstancesFromSdfFileSearchingInRosPackages;
using parsers::urdf::AddModelInstanceFromUrdfFileSearchingInRosPackages;
using systems::ConstantVectorSource;
using systems::Context;
using systems::ContinuousState;
using systems::Diagram;
using systems::DiagramBuilder;
using systems::DrakeVisualizer;
using systems::RigidBodyPlant;
using systems::Simulator;
using systems::VectorBase;

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

  parsers::PackageMap package_map;
  AddExamplePackages(&package_map);
  if (extension == "urdf") {
    table = AddModelInstanceFromUrdfFileSearchingInRosPackages(
        GetDrakePath() + model_map_[model_name], package_map,
        floating_base_type, weld_to_frame, rigid_body_tree_.get());

  } else if (extension == "sdf") {
    table = AddModelInstancesFromSdfFileSearchingInRosPackages(
        GetDrakePath() + model_map_[model_name], package_map,
        floating_base_type, weld_to_frame, rigid_body_tree_.get());
  }
  const int model_instance_id = table.begin()->second;
  return model_instance_id;
}

template <typename T>
void IiwaWorldSimBuilder<T>::AddGround() {
  DRAKE_DEMAND(!built_);
  AddFlatTerrainToWorld(rigid_body_tree_.get());
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
