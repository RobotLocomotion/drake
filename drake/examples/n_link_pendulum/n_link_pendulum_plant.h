#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace examples {
namespace n_link_pendulum {

template<typename T>
class NLinkPendulumPlant : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(NLinkPendulumPlant)

  /// Constructs an n-link pendulum plant with no GeometrySystem model.
  NLinkPendulumPlant(double mass, double length, double radius, int num_links);

  NLinkPendulumPlant(
      double mass, double length, double radius, int num_links,
      geometry::SourceId source_id,
      const std::vector<geometry::FrameId>& body_index_to_frame_id_map);

  /// Constructor that registers this model's geometry for visualization
  /// purposes only.
  NLinkPendulumPlant(double mass, double length, double radius, int num_links,
                     geometry::GeometrySystem<double>* geometry_system);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit NLinkPendulumPlant(const NLinkPendulumPlant<U>&);

  double get_mass() const { return mass_; }

  double get_length() const { return length_; }

  double get_radius() const { return radius_; }

  double get_num_links() const { return num_links_; }

  geometry::SourceId source_id() const { return source_id_; }

  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  void SetStraightAtAnAngle(systems::Context<T>*, const T& angle) const;

 protected:
  // No inputs implies no feedthrough; this makes it explicit.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

 protected:
  T DoCalcKineticEnergy(const systems::Context<T>& context) const override;
  T DoCalcPotentialEnergy(const systems::Context<T>& context) const override;

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class NLinkPendulumPlant;

  // Override of context construction so that we can delegate it to
  // MultibodyTree.
  std::unique_ptr<systems::LeafContext<T>> DoMakeContext() const override;

  void DoCalcTimeDerivatives(
      const systems::Context<T>& context,
      systems::ContinuousState<T>* derivatives) const override;

  // Helper method to build the MultibodyTree model of the system.
  void BuildMultibodyTreeModel();

  void RegisterGeometry(geometry::GeometrySystem<double>* geometry_system);

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

  double mass_;
  double length_;
  double radius_;
  int num_links_;
  multibody::MultibodyTree<T> model_;
  std::vector<const multibody::RevoluteJoint<T>*> joints_;

  // Geometry source identifier for this system to interact with geometry
  // system.
  geometry::SourceId source_id_{};

  // GeometrySystem id's ordered by BodyIndex.
  // body_index_to_frame_id_map_[world_index()] is invalid.
  std::vector<geometry::FrameId> body_index_to_frame_id_map_;

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
};

}  // namespace n_link_pendulum
}  // namespace examples
}  // namespace drake

// Disable support for symbolic evaluation.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::examples::n_link_pendulum::NLinkPendulumPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
