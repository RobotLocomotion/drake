#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to rotate relatively to one another around a
/// common axis.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frames F and M to rotate with respect to each other about
/// an axis â. The rotation angle's sign is defined such that child body B
/// rotates about axis â according to the right hand rule, with thumb aligned in
/// the axis direction.
/// Axis â is constant and has the same measures in both frames F and M, that
/// is, `â_F = â_M`.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
///
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class RevoluteJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  /// Constructor to create a revolute joint between two bodies so that
  /// frame F attached to the parent body P and frame M attached to the child
  /// body B, rotate relatively to one another about a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and rotation angle.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint limits are the pair `(-∞, ∞)`.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of revolution for this joint. Given
  ///   that frame M only rotates with respect to F and their origins are
  ///   coincident at all times, the measures of `axis` in either frame F or M
  ///   are exactly the same, that is, `axis_F = axis_M`. In other words,
  ///   `axis_F` (or `axis_M`) is the eigenvector of `R_FM` with eigenvalue
  ///   equal to one.
  ///   This vector can have any length, only the direction is used. This method
  ///   aborts if `axis` is the zero vector.
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e.
  ///   opposing motion, with ω the angular rate for `this` joint (see
  ///   get_angular_rate()).
  /// @throws std::exception if damping is negative.
  RevoluteJoint(const std::string& name,
                const Frame<T>& frame_on_parent, const Frame<T>& frame_on_child,
                const Vector3<double>& axis,
                double damping = 0) :
      RevoluteJoint<T>(name, frame_on_parent, frame_on_child, axis,
                       -std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::infinity(), damping) {}

  /// Constructor to create a revolute joint between two bodies so that
  /// frame F attached to the parent body P and frame M attached to the child
  /// body B, rotate relatively to one another about a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and rotation angle.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of revolution for this joint. Given
  ///   that frame M only rotates with respect to F and their origins are
  ///   coincident at all times, the measures of `axis` in either frame F or M
  ///   are exactly the same, that is, `axis_F = axis_M`. In other words,
  ///   `axis_F` (or `axis_M`) is the eigenvector of `R_FM` with eigenvalue
  ///   equal to one.
  ///   This vector can have any length, only the direction is used. This method
  ///   aborts if `axis` is the zero vector.
  /// @param[in] lower_limit
  ///   Lower limit, in radians, for the rotation coordinate
  ///   (see get_angle()).
  /// @param[in] upper_limit
  ///   Upper limit, in radians, for the rotation coordinate
  ///   (see get_angle()).
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e.
  ///   opposing motion, with ω the angular rate for `this` joint (see
  ///   get_angular_rate()).
  /// @throws std::exception if damping is negative.
  /// @throws std::exception if lower_limit > upper_limit.
  RevoluteJoint(const std::string& name,
                const Frame<T>& frame_on_parent, const Frame<T>& frame_on_child,
                const Vector3<double>& axis,
                double lower_limit, double upper_limit,
                double damping = 0) :
      Joint<T>(name, frame_on_parent, frame_on_child,
               VectorX<double>::Constant(1, lower_limit),
               VectorX<double>::Constant(1, upper_limit)) {
    const double kEpsilon = std::numeric_limits<double>::epsilon();
    DRAKE_DEMAND(!axis.isZero(kEpsilon));
    DRAKE_THROW_UNLESS(damping >= 0);
    DRAKE_THROW_UNLESS(lower_limit <= upper_limit);
    axis_ = axis.normalized();
    damping_ = damping;
    lower_limit_ = lower_limit;
    upper_limit_ = upper_limit;
  }

  /// Returns the axis of revolution of `this` joint as a unit vector.
  /// Since the measures of this axis in either frame F or M are the same (see
  /// this class's documentation for frames's definitions) then,
  /// `axis = axis_F = axis_M`.
  const Vector3<double>& revolute_axis() const {
    return axis_;
  }

  /// Returns `this` joint's damping constant in N⋅m⋅s.
  double damping() const { return damping_; }

  /// Returns the lower limit for `this` joint in radians.
  double lower_limit() const { return lower_limit_; }

  /// Returns the upper limit for `this` joint in radians.
  double upper_limit() const { return upper_limit_; }

  /// @name Context-dependent value access
  ///
  /// These methods require the provided context to be an instance of
  /// MultibodyTreeContext. Failure to do so leads to a std::logic_error.
  /// @{

  /// Gets the rotation angle of `this` mobilizer from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The angle coordinate of `this` joint stored in the `context`.
  const T& get_angle(const Context<T>& context) const {
    return get_mobilizer()->get_angle(context);
  }

  /// Sets the `context` so that the generalized coordinate corresponding to the
  /// rotation angle of `this` joint equals `angle`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] angle
  ///   The desired angle in radians to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const RevoluteJoint<T>& set_angle(
      Context<T>* context, const T& angle) const {
    get_mobilizer()->set_angle(context, angle);
    return *this;
  }

  void set_random_angle_distribution(const symbolic::Expression& angle) {
    get_mutable_mobilizer()->set_random_position_distribution(
        Vector1<symbolic::Expression>{angle});
  }

  /// Gets the rate of change, in radians per second, of `this` joint's
  /// angle (see get_angle()) from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The rate of change of `this` joint's angle as stored in the
  /// `context`.
  const T& get_angular_rate(const Context<T>& context) const {
    return get_mobilizer()->get_angular_rate(context);
  }

  /// Sets the rate of change, in radians per second, of this `this` joint's
  /// angle to `theta_dot`. The new rate of change `theta_dot` gets stored in
  /// `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] theta_dot
  ///   The desired rate of change of `this` joints's angle in radians per
  ///   second.
  /// @returns a constant reference to `this` joint.
  const RevoluteJoint<T>& set_angular_rate(
      Context<T>* context, const T& angle) const {
    get_mobilizer()->set_angular_rate(context, angle);
    return *this;
  }

  /// @}

  /// Adds into `forces` a given `torque` for `this` joint that is to be applied
  /// about the joint's axis. The torque is defined to be positive according to
  /// the right-hand-rule with the thumb aligned in the direction of `this`
  /// joint's axis. That is, a positive torque causes a positive rotational
  /// acceleration according to the right-hand-rule around the joint's axis.
  ///
  /// @note A torque is the moment of a set of forces whose resultant is zero.
  void AddInTorque(
      const systems::Context<T>& context,
      const T& torque,
      MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    this->AddInOneForce(context, 0, torque, forces);
  }

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a %RevoluteJoint, we must always have `joint_dof = 0` since there is
  /// only a single degree of freedom (num_velocities() == 1). `joint_tau` is
  /// the torque applied about the joint's axis, on the body declared as child
  /// (according to the revolute joint's constructor) at the origin of the child
  /// frame (which is coincident with the origin of the parent frame at all
  /// times). The torque is defined to be positive according to
  /// the right-hand-rule with the thumb aligned in the direction of `this`
  /// joint's axis. That is, a positive torque causes a positive rotational
  /// acceleration (of the child body frame) according to the right-hand-rule
  /// around the joint's axis.
  void DoAddInOneForce(
      const systems::Context<T>&,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const override {
    // Right now we assume all the forces in joint_tau go into a single
    // mobilizer.
    DRAKE_DEMAND(joint_dof == 0);
    Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> tau_mob =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  /// Joint<T> override called through public NVI, Joint::AddInDamping().
  /// Therefore arguments were already checked to be valid.
  /// This method adds into `forces` a dissipative torque according to the
  /// viscous law `τ = -d⋅ω`, with d the damping coefficient (see damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const override {
    const T damping_torque = -this->damping() * get_angular_rate(context);
    AddInTorque(context, damping_torque, forces);
  }

 private:
  int do_get_velocity_start() const override {
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const override {
    return 1;
  }

  int do_get_position_start() const override {
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const override {
    return 1;
  }

  const T& DoGetOnePosition(const systems::Context<T>& context) const override {
    return get_angle(context);
  }

  const T& DoGetOneVelocity(const systems::Context<T>& context) const override {
    return get_angular_rate(context);
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint>
  MakeImplementationBlueprint() const override;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  // Make RevoluteJoint templated on every other scalar type a friend of
  // RevoluteJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of RevoluteJoint<T>.
  template <typename> friend class RevoluteJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::RevoluteMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::RevoluteMobilizer<T>* mobilizer =
        dynamic_cast<const internal::RevoluteMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::RevoluteMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::RevoluteMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  Vector3<double> axis_;

  // This joint's damping constant in N⋅m⋅s.
  double damping_{0};

  // The lower and upper joint limits in radians.
  // lower_limit_ <= upper_limit_ always (enforced at construction).
  double lower_limit_{-std::numeric_limits<double>::infinity()};
  double upper_limit_{std::numeric_limits<double>::infinity()};
};

}  // namespace multibody
}  // namespace drake
