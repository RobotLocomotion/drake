#pragma once

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_deprecated.h"
#include "drake/multibody/rigid_body_plant/compliant_contact_model.h"
#include "drake/multibody/rigid_body_tree.h"

namespace drake {
namespace manipulation {
namespace util {

template <typename T>
struct DRAKE_DEPRECATED("2020-05-01",
    "The attic/manipulation/util package is being removed.")
ModelInstanceInfo {
  std::string absolute_model_path;
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
///
/// - double
///
template <typename T>
class DRAKE_DEPRECATED("2020-05-01",
    "The attic/manipulation/util package is being removed.")
WorldSimTreeBuilder {
 public:
  /// Constructs a WorldSimTreeBuilder object and specifies whether a call to
  /// any of the add model instance functions should compile the tree.
  ///
  /// Setting @p compile_tree to false will cause the parser to bypass tree
  /// compilation such that modifications to the recently parsed tree can be
  /// made at run time (e.g., accessing/modifying collision filters declared in
  /// the corresponding urdf/sdf file). The user is responsible for calling
  /// compile() on the tree. Take the following snippet as an example:
  /// ```
  /// auto tree_builder = std::make_unique<WorldSimTreeBuilder<double>>(false);
  /// tree_builder->StoreDrakeModel("mymodel", kModelUrdf);
  /// tree_builder->AddFixedModelInstance("mymodel", Eigen::Vector3d::Zero());
  /// auto mtree = tree_builder->mutable_tree();
  /// //  modify the tree here
  /// mtree->compile();
  /// ```
  ///
  /// @param compile_tree Specifies whether the tree should be automatically
  /// compiled. Defaults to true.
  /// @param base_tree If not null, new models will be added to this tree
  /// instead of to an empty tree.
  WorldSimTreeBuilder(bool compile_tree = true,
                      std::unique_ptr<RigidBodyTree<T>> base_tree = nullptr);

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

  /// Adds a model instance specified by its model name, @p model_name, to the
  /// `RigidBodyTree` being built. The model instance is welded to
  /// a new frame `F` constructed within this function. This new frame `F` is
  /// fixed on a body of name @p weld_to_body_name, with a transformation to
  /// this body as @p X_BF, where `B` is the body frame. The new frame is named
  /// @p frame_name. The model instance is connected to the body using a joint
  /// of type @p floating_base_type. The model name must have been previously
  /// loaded via a call to StoreModel().
  /// The function will search for the body with name @p weld_to_body_name, on
  /// the model with ID @p weld_to_body_model_instance_id.
  /// @param model_name The model with this name will be added to the tree.
  /// @param weld_to_body_name The added model will be welded to a body with
  /// this name.
  /// @param weld_to_body_model_instance_id The added model will be welded to
  /// a body with this model instance ID.
  /// @param frame_name The name of the newly added frame.
  /// @param X_BF The pose of the newly added frame `F` in the added body
  /// frame `B`.
  /// @param floating_base_type The type of the joint to weld the added model
  /// to the frame `F` on the body `B`.
  ///
  /// @return model_instance_id of the object that is added.
  int AddModelInstanceToFrame(
      const std::string& model_name, const std::string& weld_to_body_name,
      int weld_to_body_model_instance_id, const std::string& frame_name,
      const Eigen::Isometry3d& X_BF,
      const drake::multibody::joints::FloatingBaseType floating_base_type);

  /// Adds a flat terrain to the simulation.
  void AddGround();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  WorldSimTreeBuilder(const WorldSimTreeBuilder<T>& other) = delete;
  WorldSimTreeBuilder& operator=(const WorldSimTreeBuilder<T>& other) = delete;
#pragma GCC diagnostic pop

  /// Adds a model to the internal model database. Models are
  /// described by @p model_name coupled with URDF/SDF paths in @p
  /// absolute_model_path. Instances of these models can then be added
  /// to the world via the various `AddFoo()` methods provided by this
  /// class. Note that @p model_name is user-selectable but must be
  /// unique among all of the models that are stored.
  ///
  /// @see AddObjectToFrame
  /// @see AddFloatingObject
  /// @see AddFixedObject
  void StoreModel(const std::string& model_name,
                  const std::string& absolute_model_path);

  /// Like StoreModel, but uses FindResourceOrThrow to search inside
  /// the drake resource search path.
  void StoreDrakeModel(const std::string& model_name,
                       const std::string& model_path);

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

  /// Returns a pointer to the (not yet built) mutable tree. This is useful when
  /// one needs to programmatically modify a recently parsed, uncompiled tree
  /// (e.g., accessing/modifying collision filters declared in the corresponding
  /// urdf/sdf file).
  /// @pre Build() must not have been called yet.
  RigidBodyTree<T>* mutable_tree() {
    DRAKE_DEMAND(built_ == false && rigid_body_tree_ != nullptr);

    if (compile_tree_) {
      throw std::runtime_error(
          "WorldSimTreeBuilder::mutable_tree(): "
              "Attempting to return a mutable tree on an already "
              "compiled tree.");
    }
    return rigid_body_tree_.get();
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  ModelInstanceInfo<T> get_model_info_for_instance(int id) {
    return instance_id_to_model_info_.at(id);
  }
#pragma GCC diagnostic pop

  /// The compliant contact model parameters to use with the default material
  /// parameters; these values should be passed to the plant.
  systems::CompliantContactModelParameters contact_model_parameters() const {
    return contact_model_parameters_;
  }

  systems::CompliantMaterial default_contact_material() const {
    return default_contact_material_;
  }

 private:
  std::unique_ptr<RigidBodyTree<T>> rigid_body_tree_{};

  bool built_{false};
  bool compile_tree_{true};

  // Maps between models (stored as filename strings in the map) and their
  // user-supplied names (keys in the map). Instances of these models can be
  // loaded into the simulation.
  std::map<std::string, std::string> model_map_;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  std::map<int, ModelInstanceInfo<T>> instance_id_to_model_info_;
#pragma GCC diagnostic pop

  // The default parameters for evaluating contact: the parameters for the
  // model as well as the contact materials of the collision elements.
  systems::CompliantContactModelParameters contact_model_parameters_;
  systems::CompliantMaterial default_contact_material_;
};

}  // namespace util
}  // namespace manipulation
}  // namespace drake
