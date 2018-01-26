#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

#include "drake/common/drake_optional.h"
#include "drake/common/nice_type_name.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

template<typename T>
class MultibodyPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant)

 public:
  /// Default constructor creates a plant with a single "world" body.
  /// Therefore, right after creation, num_bodies() returns one.
  MultibodyPlant();

  /// Constructor for an multibody plant model for which geometry has been
  /// previously registered with `geometry_system` (for visualization only?
  /// consider templating on <T>).
  MultibodyPlant(
      geometry::SourceId source_id,
      std::unordered_map<std::string, geometry::FrameId>& frame_ids);

  int num_bodies() const {
    return model_->get_num_bodies();
  }

  int num_positions() const { return model_->get_num_positions(); }
  int num_velocities() const { return model_->get_num_velocities(); }
  int num_states() const { return model_->get_num_states(); }

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit MultibodyPlant(const MultibodyPlant<U>&);

  /// @name Methods to add new multibody elements.
  /// @{
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_E) {
    // An exception is thrown if a body named `name` was already exists.
    DRAKE_THROW_UNLESS(!HasBodyNamed(name));
    const RigidBody<T>& body = model_->template AddBody<RigidBody>(M_BBo_E);
    body_name_to_index_[name] = body.get_index();
    return body;
  }

  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<Isometry3<double>>& X_PF,
      const Body<T>& child, const optional<Isometry3<double>>& X_BM,
      Args&&... args) {
    // Make sure there is no other joint with the same name.
    DRAKE_THROW_UNLESS(!HasJointNamed(name));
    const JointType<T>& joint = model_->template AddJoint<JointType>(
        name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
    joint_name_to_index_[name] = joint.get_index();
    return joint;
  };

  template<template<typename Scalar> class ForceElementType, typename... Args>
  const ForceElementType<T>& AddForceElement(Args&&... args) {
    return model_->template AddForceElement<ForceElementType>(
        std::forward<Args>(args)...);
  }
  /// @}

  /// @name Methods to retrieve multibody elements by name.
  /// @{

  /// @returns `true` if there exists a body named `name` in the model.
  bool HasBodyNamed(const std::string& name) {
    return body_name_to_index_.find(name) != body_name_to_index_.end();
  }

  bool HasJointNamed(const std::string& name) {
    return joint_name_to_index_.find(name) != joint_name_to_index_.end();
  }

  const Body<T>& GetBodyByName(const std::string& name) const {
    auto it = body_name_to_index_.find(name);
    if (it == body_name_to_index_.end()) {
      throw std::logic_error("There is no body named '" + name +
          "' in the model.");
    }
    return model_->get_body(it->second);
  }

  const Joint<T>& GetJointByName(const std::string& name) const {
    auto it = joint_name_to_index_.find(name);
    if (it == joint_name_to_index_.end()) {
      throw std::logic_error("There is no joint named '" + name +
          "' in the model.");
    }
    return model_->get_joint(it->second);
  }

  template <template<typename> class JointType>
  const JointType<T>& GetJointByName(const std::string& name) const {
    const JointType<T>* joint =
        dynamic_cast<const JointType<T>*>(&GetJointByName(name));
    if (joint == nullptr) {
      throw std::logic_error("Joint '" + name + "' is not of type '" +
          NiceTypeName::Get<JointType<T>>() + "'.");
    }
    return *joint;
  }

  /// @}

  /// Register geometry for `body`.
  /// 1. If not done yet, register this plant as a source for the given GS.
  /// 2. Register frame for this body if not already done so.
  /// 3. Register geomtery for the corresponding FrameId.
  void RegisterGeometry(
      const Body<T>& body,
      const Isometry3<double>& X_BG, const geometry::Shape& shape,
      geometry::GeometrySystem<double>* geometry_system);

  void RegisterAnchoredGeometry(
      const Isometry3<double>& X_WG, const geometry::Shape& shape,
      geometry::GeometrySystem<double>* geometry_system);

  /// Returns a constant reference to the *world* body.
  const RigidBody<T>& get_world_body() const {
    return model_->get_world_body();
  }

  /// Returns the unique id identifying this plant as a source for a
  /// GeometrySystem.
  /// Returns `nullopt` if `this` plant did not register any geometry.
  optional<geometry::SourceId> get_source_id() const {
    return source_id_;
  }

  /// Returns the output port of frame id's used to communicate poses to a
  /// GeometrySystem. It throws a std::out_of_range exception if this system was
  /// not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_ids_output_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// GeometrySystem. It throws a std::out_of_range exception if this system was
  /// not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_poses_output_port() const;

//  const systems::InputPortDescriptor<T>& get_input_port() const;

  const systems::OutputPort<T>& get_state_output_port() const;

  /// Sets the `state` so that all the generalized positions and velocities are
  /// zero.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    model_->SetDefaultState(context, state);
  }

  bool is_finalized() const { return model_->topology_is_valid(); }

  void Finalize();

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPlant;

  // No inputs implies no feedthrough; this makes it explicit.
  // TODO(amcastro-tri): add input ports for actuators.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  void DeclareStateAndPorts();

  // It creates an empty context with no state or parameters.
  // This override gives System::AllocateContext() the chance to create a more
  // specialized context type, in this case, a MultibodyTreeContext.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // maybe geometry_source_is_registered_with(const GeometrySystem<T>& gs) ???
  bool geometry_source_is_registered() const {
    return !!source_id_;
  }

  bool body_has_registered_frame(const Body<T>& body) const {
    return body_index_to_frame_id_.find(body.get_index()) !=
        body_index_to_frame_id_.end();
  }

  // Helper method to declare output ports used by this plant to communicate
  // with a GeometrySystem.
  void DeclareGeometrySystemPorts();

  // Allocate the id output.
  geometry::FrameIdVector AllocateFrameIdOutput(
      const systems::Context<T>& context) const;

  // Calculate the id output.
  void CalcFrameIdOutput(
      const systems::Context<T>& context,
      geometry::FrameIdVector* id_set) const;

  // Allocate the frame pose set output port value.
  geometry::FramePoseVector<T> AllocateFramePoseOutput(
      const systems::Context<T>& context) const;

  // Calculate the frame pose set output port value.
  void CalcFramePoseOutput(const systems::Context<T>& context,
                           geometry::FramePoseVector<T>* poses) const;

  // Copies the state in `context` to `output`.
  void CopyStateOut(const systems::Context<T>& context,
                    systems::BasicVector<T>* output) const;

  // The entire multibody model.
  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  optional<geometry::SourceId> source_id_{nullopt};

  // Map used to find body indexes by body name.
  std::unordered_map<std::string, BodyIndex> body_name_to_index_;

  // Map used to find joint indexes by joint name.
  std::unordered_map<std::string, JointIndex> joint_name_to_index_;

  // Frame Id's for each body in the model:
  // Not all bodies need to be in this map.
  std::unordered_map<BodyIndex, geometry::FrameId> body_index_to_frame_id_;

  // Map provided at construction that tells how bodies (referenced by name),
  // map to frame ids.
  std::unordered_map<std::string, geometry::FrameId> body_name_to_frame_id_;

  // Port handles for geometry:
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};

  // Port handles for general input/output:
  int applied_torque_input_{-1};
  int state_output_port_{-1};

};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::multibody::multibody_plant::MultibodyPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
