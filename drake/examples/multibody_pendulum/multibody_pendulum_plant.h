#pragma once

#include <memory>

#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace examples {
namespace multibody_pendulum {

/// A model of an idealized pendulum with a point mass on the end of a massless
/// cord.
/// The pendulum oscillates in the x-z plane with its revolute axis coincident
/// with the y-axis. Gravity points downwards in the -z axis direction.
///
/// The parameters of the plant are:
/// - mass: the mass of the idealized point mass.
/// - length: the length of the massless cord on which the mass is suspended.
/// - gravity: the acceleration of gravity.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template<typename T>
class MultibodyPendulumPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPendulumPlant)

  /// Constructs a model of an idealized pendulum of a given `mass` and
  /// suspended by a massless cord with a given `length`. Gravity points in the
  /// `-z` axis direction with the acceleration constant `gravity`.
  MultibodyPendulumPlant(double mass, double length, double gravity);

  /// Constructs a plant given a SourceId to communicate with a GeometrySystem.
  /// FrameId identifies the body frame of `this` pendulum in that
  /// GeometrySystem.
  /// See this class's documentation for a description of the physical
  /// parameters.
  MultibodyPendulumPlant(
      double mass, double length, double gravity,
      geometry::SourceId source_id, geometry::FrameId frame_id);

  /// This constructor registers `this` plant as a source for `geometry_system`.
  /// The constructor will registrate frames and geometry for visualization with
  /// GeometrySystem.
  /// See this class's documentation for a description of the physical
  /// parameters.
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

  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  /// Sets the state for this system in `context` to be that of `this` pendulum
  /// at a given `angle`. Mainly used to set initial conditions.
  void SetAngle(systems::Context<T>*, const T& angle) const;

  /// Computes the period for the small oscillations of a simple pendulum.
  /// The solution assumes no damping. Useful to compute a reference time scale.
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
