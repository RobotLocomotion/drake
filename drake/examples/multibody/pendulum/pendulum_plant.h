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
namespace multibody {
namespace pendulum {

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
  /// There will be no geometry associated to the model.
  MultibodyPendulumPlant(double mass, double length, double gravity);

  /// Constructs a model of an idealized pendulum wiht parameters as described
  /// in this class's documentation.
  /// This constructor registers `this` plant as a source for `geometry_system`
  /// as well as the frames and geometry used for visualization.
  ///
  /// See this class's documentation for a description of the physical
  /// parameters.
  MultibodyPendulumPlant(
      double mass, double length, double gravity,
      geometry::GeometrySystem<double>* geometry_system);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit MultibodyPendulumPlant(const MultibodyPendulumPlant<U>&);

  /// Returns the mass of this point pendulum.
  double get_mass() const { return mass_; }

  /// Returns the length of the cord from which the point mass is suspended.
  double get_length() const { return length_; }

  /// Returns the value of the acceleration of gravity in the model.
  double get_gravity() const { return gravity_; }

  /// Returns the unique id identifying this plant as a source for a
  /// GeometrySystem.
  /// The returned id will be invalid when the plant is not registered with any
  /// GeometrySystem.
  geometry::SourceId get_source_id() const { return source_id_; }

  /// Returns the output port of frame id's used to communicate poses to a
  /// GeometrySystem. It throws a std::out_of_range exception is this system was
  /// not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  /// Returns the output port of frames's poses to communicate with a
  /// GeometrySystem. It throws a std::out_of_range exception is this system was
  /// not registered with a GeometrySystem.
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

  // Constructs a plant given a SourceId to communicate with a GeometrySystem.
  // FrameId identifies the frame of the one and only body in the pendulum's
  // model, which is defined to have its origin coincident with the world's
  // frame W origin.
  // See this class's documentation for a description of the physical
  // parameters.
  MultibodyPendulumPlant(
      double mass, double length, double gravity,
      geometry::SourceId source_id, geometry::FrameId frame_id);

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

  // The physical parameters of the model:
  double mass_{1.0};
  double length_{0.5};
  double gravity_{9.81};

  // The entire multibody model.
  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;

  // The one and only body in this model.
  const drake::multibody::RigidBody<T>* link_;

  // The one and only joint in this model.
  const drake::multibody::RevoluteJoint<T>* joint_;

  // Geometry source identifier for this system to interact with geometry
  // system.
  geometry::SourceId source_id_{};

  // The frame Id associated with the only Body in the model.
  geometry::FrameId frame_id_;

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
};

}  // namespace pendulum
}  // namespace multibody
}  // namespace examples
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::examples::multibody::pendulum::MultibodyPendulumPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
