#include "drake/manipulation/util/world_sim_tree_builder.h"

#include <algorithm>
#include <map>
#include <utility>

#include <spruce.hh>

#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/model_instance_id_table.h"
#include "drake/multibody/parsers/sdf_parser.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_frame.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"

using Eigen::aligned_allocator;
using Eigen::Vector3d;
using drake::multibody::joints::kQuaternion;
using std::allocate_shared;
using std::string;

namespace drake {
namespace manipulation {
namespace util {

template <typename T>
WorldSimTreeBuilder<T>::WorldSimTreeBuilder(bool compile_tree)
    : compile_tree_(compile_tree) {
  // TODO(SeanCurtis-TRI): These values preserve the historical behavior of the
  // compliant contact model. However, it has several issues:
  //  1. Young's modulus is far too small (it does not reflect a reasonable
  //     value for a real material).
  //     - the characteristic area is too large for the scenario, but must be
  //       this large to offset the small Young's modulus.
  //  2. the dissipation value is not a realistic value for the Hunt-Crossley
  //     model. According to the original paper, it shouldn't be much larger
  //     than 0.6 or so.
  //  3. The Young's modulus value (20000) is twice as big as the old hard-coded
  //     default. This is because,  *before*, it was a model parameter. Now it
  //     is a material property. And the combination of two identical material
  //     Young's modulus values produces an effective Young's modulus half as
  //     large.
  contact_model_parameters_.v_stiction_tolerance = 0.01;  // m/s
  contact_model_parameters_.characteristic_radius = 1.0;  // m^2
  default_contact_material_.set_youngs_modulus(20000);    // Pa
  default_contact_material_.set_dissipation(2);           // s/m
  default_contact_material_.set_friction(0.9, 0.5);
}

template <typename T>
WorldSimTreeBuilder<T>::~WorldSimTreeBuilder() {}

template <typename T>
int WorldSimTreeBuilder<T>::AddFixedModelInstance(const string& model_name,
                                                  const Vector3d& xyz,
                                                  const Vector3d& rpy) {
  DRAKE_DEMAND(!built_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame<T>>(
      aligned_allocator<RigidBodyFrame<T>>(), "world", nullptr, xyz, rpy);

  return AddModelInstanceToFrame(model_name, weld_to_frame);
}

template <typename T>
int WorldSimTreeBuilder<T>::AddFloatingModelInstance(const string& model_name,
                                                     const Vector3d& xyz,
                                                     const Vector3d& rpy) {
  DRAKE_DEMAND(!built_);

  auto weld_to_frame = allocate_shared<RigidBodyFrame<T>>(
      aligned_allocator<RigidBodyFrame<T>>(), "world", nullptr, xyz, rpy);

  return AddModelInstanceToFrame(model_name, weld_to_frame, kQuaternion);
}

template <typename T>
int WorldSimTreeBuilder<T>::AddModelInstanceToFrame(
    const string& model_name, const string& weld_to_body_name,
    const int weld_to_body_model_instance_id,
    const string& frame_name, const Eigen::Isometry3d& X_BF,
    const drake::multibody::joints::FloatingBaseType floating_base_type) {
  // Create a new frame.
  auto weld_to_frame = std::make_shared<RigidBodyFrame<T>>(
      frame_name,
      rigid_body_tree_->get_mutable_body(rigid_body_tree_->FindBodyIndex(
          weld_to_body_name, weld_to_body_model_instance_id)), X_BF);
  rigid_body_tree_->addFrame(weld_to_frame);
  return AddModelInstanceToFrame(model_name, weld_to_frame, floating_base_type);
}

template <typename T>
int WorldSimTreeBuilder<T>::AddModelInstanceToFrame(
    const string& model_name, std::shared_ptr<RigidBodyFrame<T>> weld_to_frame,
    const drake::multibody::joints::FloatingBaseType floating_base_type) {
  DRAKE_DEMAND(!built_);

  spruce::path p(model_map_[model_name]);

  // Converts the file extension to be lower case.
  auto extension = p.extension();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 ::tolower);

  parsers::ModelInstanceIdTable table;

  DRAKE_DEMAND(extension == ".urdf" || extension == ".sdf");
  if (extension == ".urdf") {
    table = drake::parsers::urdf::AddModelInstanceFromUrdfFile(
        model_map_[model_name], floating_base_type, weld_to_frame,
        compile_tree_, rigid_body_tree_.get());

  } else if (extension == ".sdf") {
    table = drake::parsers::sdf::AddModelInstancesFromSdfFile(
        model_map_[model_name], floating_base_type, weld_to_frame,
        compile_tree_, rigid_body_tree_.get());
  }
  const int model_instance_id = table.begin()->second;

  ModelInstanceInfo<T> info;
  info.absolute_model_path = model_map_[model_name];
  info.instance_id = model_instance_id;
  info.world_offset = weld_to_frame;
  instance_id_to_model_info_[model_instance_id] = info;
  return model_instance_id;
}

template <typename T>
void WorldSimTreeBuilder<T>::AddGround() {
  DRAKE_DEMAND(!built_);
  drake::multibody::AddFlatTerrainToWorld(rigid_body_tree_.get());
}

template <typename T>
void WorldSimTreeBuilder<T>::StoreModel(
    const std::string& model_name, const std::string& absolute_model_path) {
  DRAKE_DEMAND(model_map_.find(model_name) == model_map_.end());
  model_map_.insert(
      std::pair<std::string, std::string>(model_name, absolute_model_path));
}

template <typename T>
void WorldSimTreeBuilder<T>::StoreDrakeModel(const std::string& model_name,
                                             const std::string& model_path) {
  StoreModel(model_name, FindResourceOrThrow(model_path));
}

template class WorldSimTreeBuilder<double>;

}  // namespace util
}  // namespace manipulation
}  // namespace drake
