#pragma once

#include <limits>
#include <memory>

#include "drake/common/drake_optional.h"
#include "drake/examples/multibody/acrobot/gen/acrobot_state.h"
#include "drake/geometry/geometry_system.h"
#include "drake/multibody/multibody_tree/force_element.h"
#include "drake/multibody/multibody_tree/joints/revolute_joint.h"
#include "drake/multibody/multibody_tree/multibody_tree.h"
#include "drake/multibody/multibody_tree/rigid_body.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace examples {
namespace multibody {
namespace acrobot {

/// A model of an idealized acrobot with a point mass on the end of a massless
/// rigid rod.
/// The acrobot oscillates in the x-z plane with its revolute axis coincident
/// with the y-axis. Gravity points downwards in the -z axis direction.
///
/// The parameters of the plant are:
/// - mass: the mass of the idealized point mass.
/// - length: the length of the massless rod on which the mass is suspended.
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
class AcrobotPlant final : public systems::LeafSystem<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AcrobotPlant)

  /// Constructs a model of an idealized acrobot of a given `mass` and
  /// suspended by a massless rigid rod with a given `length`. Gravity points in
  /// the `-z` axis direction with the acceleration constant `gravity`.
  /// This constructor doesn't register any geometry.
  AcrobotPlant(double mass, double length, double gravity);

  /// Constructs a model of an idealized acrobot of a given `mass` and
  /// suspended by a massless rigid rod with a given `length`. Gravity points in
  /// the `-z` axis direction with the acceleration constant `gravity`.
  /// This constructor registers `this` plant as a source for `geometry_system`
  /// as well as the frames and geometry used for visualization.
  AcrobotPlant(
      double mass, double length, double gravity,
      geometry::GeometrySystem<double>* geometry_system);

  /// Scalar-converting copy constructor.
  template <typename U>
  explicit AcrobotPlant(const AcrobotPlant<U>&);

  /// Returns the mass of this point acrobot.
  double mass() const { return mass_; }

  /// Returns the length of the rod from which the point mass is suspended.
  double length() const { return length_; }

  /// Returns the value of the acceleration of gravity in the model.
  double gravity() const { return gravity_; }

  /// Evaluates the input port and returns the scalar value
  /// of the commanded torque.
  const T& get_tau(const systems::Context<T>& context) const {
    const Eigen::VectorBlock<const VectorX<T>> input =
        this->template EvalEigenVectorInput(context, applied_torque_input_);
    DRAKE_DEMAND(input.size() == 1);
    return input.coeff(0);
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
  const systems::OutputPort<T>& get_geometry_id_output_port() const;

  /// Returns the output port of frames' poses to communicate with a
  /// GeometrySystem. It throws a std::out_of_range exception if this system was
  /// not registered with a GeometrySystem.
  const systems::OutputPort<T>& get_geometry_pose_output_port() const;

  const systems::InputPortDescriptor<T>& get_input_port() const;

  const systems::OutputPort<T>& get_state_output_port() const;

  /// Sets the state for this system in `context` to be that of `this` acrobot
  /// at a given `angle`, in radians. Mainly used to set initial conditions.
  void SetAngle(systems::Context<T>*, const T& angle) const;

  /// Computes the period for the small oscillations of a simple acrobot.
  /// The solution assumes no damping. Useful to compute a reference time scale.
  static T CalcSimpleAcrobotPeriod(const T& length, const T& gravity) {
    DRAKE_DEMAND(length > 0);
    DRAKE_DEMAND(gravity > 0);
    using std::sqrt;
    return 2.0 * M_PI * sqrt(length / gravity);
  }

  /// Sets the state in `context` so that generalized positions and velocities
  /// are zero. For quaternion based joints the quaternion is set to be the
  /// identity (or equivalently a zero rotation).
  void SetDefaultState(const systems::Context<T>& context,
                       systems::State<T>* state) const override {
    DRAKE_DEMAND(state != nullptr);
    model_->SetDefaultState(context, state);
  }

 private:
  // Allow different specializations to access each other's private data for
  // scalar conversion.
  template <typename U> friend class AcrobotPlant;

  // Helper method for NaN initialization.
  static constexpr T nan() {
    return std::numeric_limits<
        typename Eigen::NumTraits<T>::Literal>::quiet_NaN();
  }

  // No inputs implies no feedthrough; this makes it explicit.
  optional<bool> DoHasDirectFeedthrough(int, int) const override {
    return false;
  }

  // It creates an empty context with no state or parameters.
  // This override gives System::AllocateContext() the chance to create a more
  // specialized context type, in this case, a MultibodyTreeContext.
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

  // Copies the state in `context` to `output`.
  void CopyStateOut(const systems::Context<T>& context,
                    AcrobotState<T>* output) const;

  // The physical parameters of the model. They are initialized with NaN for a
  // quick detection of uninitialized values.
  double mass_{nan()};      // In kilograms.
  double length_{nan()};    // In meters.
  double gravity_{nan()};   // In m/s².

  // The entire multibody model.
  std::unique_ptr<drake::multibody::MultibodyTree<T>> model_;

  // The one and only body in this model.
  const drake::multibody::RigidBody<T>* link_;

  // The one and only joint in this model.
  const drake::multibody::RevoluteJoint<T>* joint_;

  // Geometry source identifier for this system to interact with geometry
  // system. It is made optional for plants that do not register geometry
  // (dynamics only).
  optional<geometry::SourceId> source_id_{nullopt};

  // The frame Id associated with the only Body in the model.
  optional<geometry::FrameId> frame_id_{nullopt};

  // Port handles
  int geometry_id_port_{-1};
  int geometry_pose_port_{-1};
  int applied_torque_input_{-1};
  int state_output_port_{-1};
};

}  // namespace acrobot
}  // namespace multibody
}  // namespace examples
}  // namespace drake

// Disable support for symbolic evaluation.
// TODO(amcastro-tri): Allow symbolic evaluation once MultibodyTree supports it.
namespace drake {
namespace systems {
namespace scalar_conversion {
template <>
struct Traits<drake::examples::multibody::acrobot::AcrobotPlant> :
    public NonSymbolicTraits {};
}  // namespace scalar_conversion
}  // namespace systems
}  // namespace drake
