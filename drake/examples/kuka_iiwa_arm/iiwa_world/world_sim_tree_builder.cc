#include "drake/examples/kuka_iiwa_arm/iiwa_world/world_sim_tree_builder.h"

#include <map>
#include <utility>

#include "drake/common/drake_path.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/parser_model_instance_id_table.h"
#include "drake/multibody/parser_sdf.h"
#include "drake/multibody/parser_urdf.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using Eigen::aligned_allocator;
using Eigen::Vector3d;
using drake::multibody::joints::FloatingBaseType;
using drake::multibody::joints::kFixed;
using drake::multibody::joints::kQuaternion;
using drake::parsers::ModelInstanceIdTable;
using drake::parsers::sdf::AddModelInstancesFromSdfFile;
using drake::parsers::urdf::AddModelInstanceFromUrdfFile;
using drake::systems::Context;
using drake::systems::ContinuousState;
using drake::systems::DrakeVisualizer;
using drake::systems::VectorBase;
using std::allocate_shared;
using std::make_unique;
using std::string;
using std::unique_ptr;

namespace drake {
namespace examples {
namespace kuka_iiwa_arm {

template <typename T>
WorldSimTreeBuilder<T>::WorldSimTreeBuilder() {}

template <typename T>
WorldSimTreeBuilder<T>::~WorldSimTreeBuilder() {}

template <typename T>
int WorldSimTreeBuilder<T>::AddFixedModelInstance(const string& model_name,
                                                  const Vector3d& xyz,
                                                  const Vector3d& rpy) {
  DRAKE_DEMAND(!built_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), "world", nullptr, xyz, rpy);

  return AddModelInstanceToFrame(model_name, xyz, rpy, weld_to_frame);
}

template <typename T>
int WorldSimTreeBuilder<T>::AddFloatingModelInstance(const string& model_name,
                                                     const Vector3d& xyz,
                                                     const Vector3d& rpy) {
  DRAKE_DEMAND(!built_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame>(
      aligned_allocator<RigidBodyFrame>(), "world", nullptr, xyz, rpy);

  return AddModelInstanceToFrame(model_name, xyz, rpy, weld_to_frame,
                                 kQuaternion);
}

template <typename T>
int WorldSimTreeBuilder<T>::AddModelInstanceToFrame(
    const string& model_name, const Vector3d& xyz, const Vector3d& rpy,
    std::shared_ptr<RigidBodyFrame> weld_to_frame,
    const drake::multibody::joints::FloatingBaseType floating_base_type) {
  DRAKE_DEMAND(!built_);
  std::size_t extension_location = model_map_[model_name].find_last_of(".");

  DRAKE_DEMAND(extension_location < model_map_[model_name].size());

  std::string extension = model_map_[model_name].substr(extension_location + 1);

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
void WorldSimTreeBuilder<T>::AddGround() {
  DRAKE_DEMAND(!built_);
  drake::multibody::AddFlatTerrainToWorld(rigid_body_tree_.get());
}

template <typename T>
void WorldSimTreeBuilder<T>::StoreModel(const std::string& model_name,
                                        const std::string& model_path) {
  DRAKE_DEMAND(model_map_.find(model_name) == model_map_.end());
  model_map_.insert(
      std::pair<std::string, std::string>(model_name, model_path));
}

template class WorldSimTreeBuilder<double>;

}  // namespace kuka_iiwa_arm
}  // namespace examples
}  // namespace drake
