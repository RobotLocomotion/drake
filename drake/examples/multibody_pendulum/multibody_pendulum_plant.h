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
namespace multibody_pendulum {

template<typename T>
class MultibodyPendulumPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPendulumPlant)

  /// Constructs an n-link pendulum plant with no GeometrySystem model.
  MultibodyPendulumPlant(double mass, double length, double gravity);

  MultibodyPendulumPlant(
      double mass, double length, double gravity,
      geometry::SourceId source_id, geometry::FrameId frame_id);

  /// Constructor that registers this model's geometry for visualization
  /// purposes only.
  MultibodyPendulumPlant(
      double mass, double length, double gravity,
      geometry::GeometrySystem<double>* geometry_system);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit MultibodyPendulumPlant(const MultibodyPendulumPlant<U>&);

  double get_mass() const { return mass_; }

  double get_length() const { return length_; }

  double get_gravity() const { return gravity_; }

  geometry::SourceId get_source_id() const { return source_id_; }

  const multibody::MultibodyTree<T>& get_multibody_model() const {
    return *model_;
  }

  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  void SetAngle(systems::Context<T>*, const T& angle) const;

  /// Computes the period for the small oscillations of a simple pendulum.
  /// The solution assumes no damping.
  static T SimplePendulumPeriod(const T& length, const T& gravity) {
    DRAKE_DEMAND(length > 0);
    DRAKE_DEMAND(gravity > 0);
    using std::sqrt;
    return 2.0 * M_PI * sqrt(length / gravity);
  }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class MultibodyPendulumPlant;

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

  double mass_{1.0};
  double length_{0.5};
  double damping_{0.1};
  double gravity_{9.81};

  // The entire multibody model.
  std::unique_ptr<multibody::MultibodyTree<T>> model_;

  // The one and only body in this model.
  const multibody::RigidBody<T>* link_;

  // The one and only joint in this model.
  const multibody::RevoluteJoint<T>* joint_;

  // Geometry source identifier for this system to interact with geometry
  // system.
  geometry::SourceId source_id_{};

  // The frame Id associated with the only Body in the model.
  geometry::FrameId frame_id_;

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
};

}  // namespace multibody_pendulum
}  // namespace examples
}  // namespace drake

// Disable support for symbolic evaluation.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::examples::multibody_pendulum::MultibodyPendulumPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
