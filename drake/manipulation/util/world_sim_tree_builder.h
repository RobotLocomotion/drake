#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace util {

template<typename T> struct ModelInstanceInfo {
  std::string model_path;
  int instance_id;
  std::shared_ptr<RigidBodyFrame<T>> world_offset;
};

/// A helper class to construct robot world RigidBodyTree objects from model
/// (URDF/SDF) files. Models (e.g., robots, objects for manipulation, etc.)
/// can be stored and added to the tree to be built.
///
/// @tparam T must be a valid Eigen ScalarType.
///
/// Instantiated templates for the following ScalarTypes are provided:
/// - double
///
template <typename T>
class WorldSimTreeBuilder {
 public:
  WorldSimTreeBuilder();

  ~WorldSimTreeBuilder();

  /// Adds a fixed model instance specified by its name, @p model_name, to the
  /// `RigidBodyTree` being built at the pose specified by position @p xyz and
  /// orientation @p rpy. The model name must have been previously loaded via
  /// a call to StoreModel().
  ///
  /// @return model_instance_id of the object that is added.
  int AddFixedModelInstance(
      const std::string& model_name, const Eigen::Vector3d& xyz,
      const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /// Adds a floating model instance specified by its name, @p model_name, to
  /// the `RigidBodyTree` being built at the pose specified by position @p xyz
  /// and orientation @p rpy. The model name must have been previously loaded
  /// via a call to StoreModel().
  ///
  /// @return model_instance_id of the object that is added.
  int AddFloatingModelInstance(
      const std::string& model_name, const Eigen::Vector3d& xyz,
      const Eigen::Vector3d& rpy = Eigen::Vector3d::Zero());

  /// Adds a model instance specified by its model name, @p model_name, to
  /// the `RigidBodyTree` being built. The model instance is connected to the
  /// existing world based on a @p weld_to_frame using a joint of type @p
  /// floating_base_type. The model name must have been previously loaded via
  /// a call to StoreModel().
  ///
  /// @return model_instance_id of the object that is added.
  int AddModelInstanceToFrame(
      const std::string& model_name,
      std::shared_ptr<RigidBodyFrame<T>> weld_to_frame,
      const drake::multibody::joints::FloatingBaseType floating_base_type =
          drake::multibody::joints::kFixed);

  /// Adds a flat terrain to the simulation.
  void AddGround();

  // We are neither copyable nor moveable.
  WorldSimTreeBuilder(const WorldSimTreeBuilder<T>& other) = delete;
  WorldSimTreeBuilder& operator=(const WorldSimTreeBuilder<T>& other) = delete;

  /// Adds a model to the internal model database. Models are described by
  /// @p model_name coupled with URDF/SDF paths in @p model_path. Instances
  /// of these models can then be added to the world via the various
  /// `AddFoo()` methods provided by this class. Note that @p model_name is
  /// user-selectable but must be unique among all of the models that are
  /// stored.
  ///
  /// @see AddObjectToFrame
  /// @see AddFloatingObject
  /// @see AddFixedObject
  void StoreModel(const std::string& model_name, const std::string& model_path);

  /// Gets a unique pointer to the `RigidBodyTree` that was built.
  /// This method can only be called if it was not previously called.
  /// Ownership of the manufactured RigidBodyTree is transferred to the
  /// calling code. The instance of this class should be discarded after this
  /// method is called.
  std::unique_ptr<RigidBodyTree<T>> Build(void) {
    DRAKE_DEMAND(built_ == false && rigid_body_tree_ != nullptr);
    built_ = true;
    return std::move(rigid_body_tree_);
  }

  /// Returns the (not yet built) tree.
  /// @pre Build() must not have been called yet.
  const RigidBodyTree<T>& tree() const {
    DRAKE_DEMAND(built_ == false && rigid_body_tree_ != nullptr);
    return *rigid_body_tree_;
  }

  ModelInstanceInfo<T> get_model_info_for_instance(int id) {
    return instance_id_to_model_info_.at(id);
  }

  /// The compliant contact model parameters to use with the default material
  /// parameters; these values should be passed to the plant.
  systems::CompliantContactModelParameters contact_model_parameters() const {
    return contact_model_parameters_;
  }

  systems::CompliantMaterial default_contact_material() const {
    return default_contact_material_;
  }


 private:
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{
      std::make_unique<RigidBodyTree<T>>()};

  bool built_{false};

  // Maps between models (stored as filename strings in the map) and their
  // user-supplied names (keys in the map). Instances of these models can be
  // loaded into the simulation.
  std::map<std::string, std::string> model_map_;

  std::map<int, ModelInstanceInfo<T>> instance_id_to_model_info_;

  // The default parameters for evaluating contact: the parameters for the
  // model as well as the contact materials of the collision elements.
  systems::CompliantContactModelParameters contact_model_parameters_;
  systems::CompliantMaterial default_contact_material_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
