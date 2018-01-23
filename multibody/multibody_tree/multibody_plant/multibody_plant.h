#pragma once

#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

#include "drake/common/drake_optional.h"
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

/// The Acrobot - a canonical underactuated system as described in <a
/// href="http://underactuated.mit.edu/underactuated.html?chapter=3">Chapter 3
/// of Underactuated Robotics</a>.
/// This plant is modeled using a MultibodyTree.
///
/// @tparam T The vector element type, which must be a valid Eigen scalar.
/// @param m1 Mass of link 1 (kg).
/// @param m2 Mass of link 2 (kg).
/// @param l1 Length of link 1 (m).
/// @param l2 Length of link 2 (m).
/// @param lc1 Vertical distance from shoulder joint to center of mass of
/// link 1 (m).
/// @param lc2 Vertical distance from elbow joint to center of mass of
/// link 2 (m).
/// @param Ic1 Inertia of link 1 about the center of mass of link 1
/// (kg*m^2).
/// @param Ic2 Inertia of link 2 about the center of mass of link 2
/// (kg*m^2).
/// @param b1 Damping coefficient of the shoulder joint (kg*m^2/s).
/// @param b2 Damping coefficient of the elbow joint (kg*m^2/s).
/// @param g Gravitational constant (m/s^2).
///
/// The parameters are defaulted to values in Spong's paper (see
/// acrobot_spong_controller.cc for more details).
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
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

  // NOTE: it'd be nice to have a get_bodies() method so we can do range loops
  // like in:
  // for (const auto& body : plant.get_bodies())
  //     body.do_something();
  //
  // Look at code for GeometryState::get_frame_ids() for ideas.

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit MultibodyPlant(const MultibodyPlant<U>&);

#if 0
  /// @name Methods to add new multibody elements.
  /// @{
  const RigidBody<T>& AddRigidBody(
      const std::string& name, const SpatialInertia<double>& M_BBo_E) {
    return model_->template AddBody<RigidBody>(M_BBo_E);
  }

  template<template<typename> class JointType, typename... Args>
  const JointType<T>& AddJoint(
      const std::string& name,
      const Body<T>& parent, const optional<Isometry3<double>>& X_PF,
      const Body<T>& child, const optional<Isometry3<double>>& X_BM,
      Args&&... args) {
    return model_->template AddJoint(
        name, parent, X_PF, child, X_BM, std::forward<Args>(args)...);
  };
  /// @}
#endif
  /// Register geometry for `body`.
  /// 1. If not done yet, register this plant as a source for the given GS.
  /// 2. Register frame for this body if not already done so.
  /// 3. Register geomtery for the corresponding FrameId.
#if 0
  void RegisterGeometry(
      const Body<T>& body,
      std::unique_ptr<geometry::GeometryInstance> geometry,
      geometry::GeometrySystem<double>* geometry_system);
#endif

#if 0
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

  const systems::InputPortDescriptor<T>& get_input_port() const;

  const systems::OutputPort<T>& get_state_output_port() const;

  /// Sets the state in `context` so that generalized positions and velocities
  /// are zero.
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    model_->SetDefaultState(context, state);
  }
#endif

  bool is_finalized() const { return model_->topology_is_valid(); }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPlant;

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // No inputs implies no feedthrough; this makes it explicit.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  void Finalize();

  void DeclareStateAndPorts();

  // It creates an empty context with no state or parameters.
  // This override gives System::AllocateContext() the chance to create a more
  // specialized context type, in this case, a MultibodyTreeContext.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  void RegisterGeometry(geometry::GeometrySystem<double>* geometry_system);

  // Helper method to declare output ports used by this plant to communicate
  // with a GeometrySystem.
  //void DeclareGeometrySystemPorts();

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

  // Frame Id's for each body in the model:
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

#if 0
template <>
struct Traits<drake::multibody::multibody_plant::MultibodyPlant> {
  template <typename T, typename U>
  using supported = typename std::conditional<
      FromDoubleTraits::supported<T, U>::value &&
      NonSymbolicTraits::supported<T, U>::value,
      std::true_type, std::false_type>::type;
};
#endif
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
