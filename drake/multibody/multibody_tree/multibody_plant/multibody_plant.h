#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace multibody_plant {

template<typename T>
class MultibodyPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant)

  MultibodyPlant(
      std::unique_ptr<MultibodyTree<T>> model,
      const std::string& name,
      geometry::GeometrySystem<T>* geometry_system);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit MultibodyPlant(const MultibodyPlant<U>&);

  /// Returns the number of bodies in the world.
  int get_num_bodies() const;

  /// Returns the number of generalized positions of the model.
  int get_num_positions() const;

  /// Returns the number of generalized velocities of the model.
  int get_num_velocities() const;

  /// Returns the size of the continuous state vector of the system which equals
  /// get_num_positions() plus get_num_velocities().
  int get_num_states() const;

  geometry::SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  void RegisterGeometry(
      const Body<T>& body, const Isometry3<double>& X_BG,
      std::unique_ptr<geometry::Shape> shape,
      geometry::GeometrySystem<T>* geometry_system);

 protected:
  // Constructor for subclasses wanting to programatically create their model.
  MultibodyPlant(const std::string& name);

  /// Subclasses can get a mutable reference to the MultibodyTree model
  MultibodyTree<T>& get_mutable_model() {
    return *model_;
  }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

  /// Sub-classes must provide this method to build their model for this plant.
  virtual void BuildMultibodyModel(MultibodyTree<T>* model) = 0;

  /// Sub-classes MUST call this method from their constructor.
  void Init(geometry::GeometrySystem<T>* geometry_system);

 private:
  // Method the state of this plant consistent with the Multibody model.
  // TODO(amcastro-tri): possibly make pure virtual or provide some mechanism
  // for supporting continuous and time-stepping plants.
  void DeclareState();

  // Called by Init() to setup geometry_system for this plant.
  void RegisterGeometrySystemFrames(
      geometry::GeometrySystem<T>* geometry_system);

  // For multibody systems registering geometry in a GeometrySystem, this
  // method allows to declare output ports to communicate with that
  // GeometrySystem.
  void DeclareOutputPortsForGeometrySystem(
      geometry::GeometrySystem<T>* geometry_system);

  // No inputs implies no feedthrough; this makes it explicit.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  // Override of context construction so that we can delegate it to
  // MultibodyTree.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

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

  std::unique_ptr<MultibodyTree<T>> model_;

  // Joints ordered by JointIndex.
  std::vector<const multibody::RevoluteJoint<T>*> joints_;

  // Geometry source identifier for this system to interact with geometry system
  geometry::SourceId source_id_{};
  // The id in GeometrySystem for each body in the model, order by BodyIndex.
  // id for the world body is invalid, i.e. frame_ids_[world_index()] is
  // invalid.
  std::vector<geometry::FrameId> body_index_to_frame_id_map_;

  std::vector<int> body_index_to_frame_index_map_;

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
};

}  // namespace multibody_plant
}  // namespace multibody
}  // namespace drake
