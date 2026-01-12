#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/revolute_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to rotate relatively to one another around a
/// common axis.
/// That is, given a frame Jp attached to the parent body P and a frame Jc
/// attached to the child body C (see the Joint class's documentation),
/// this Joint allows frames Jp and Jc to rotate with respect to each other
/// about an axis â. The rotation angle's sign is defined such that child body
/// C rotates about axis â according to the right hand rule, with thumb aligned
/// in the axis direction.
/// Axis vector â is constant and has the same components in both frames Jp and
/// Jc, that is, `â_Jp = â_Jc`.
///
/// @tparam_default_scalar
template <typename T>
class RevoluteJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RevoluteJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  /// Constructor to create a revolute joint between two bodies so that
  /// frame Jp attached to the parent body P and frame Jc attached to the child
  /// body C, rotate relatively to one another about a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and rotation angle.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint position, velocity and acceleration limits are the pair `(-∞, ∞)`.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of revolution for this joint. Given
  ///   that frame Jc only rotates with respect to Jp and their origins are
  ///   coincident at all times, the components of `axis` in either frame Jp or
  ///   Jc are exactly the same, that is, `axis_Jp = axis_Jc`. In other words,
  ///   `axis_Jp` (or `axis_Jc`) is the eigenvector of `R_JpJc` with eigenvalue
  ///   equal to one. This vector can have any length, only the direction is
  ///   used. This method aborts if `axis` is the zero vector.
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e.
  ///   opposing motion, with ω the angular rate for `this` joint (see
  ///   get_angular_rate()).
  /// @throws std::exception if damping is negative.
  RevoluteJoint(const std::string& name, const Frame<T>& frame_on_parent,
                const Frame<T>& frame_on_child, const Vector3<double>& axis,
                double damping = 0)
      : RevoluteJoint<T>(name, frame_on_parent, frame_on_child, axis,
                         -std::numeric_limits<double>::infinity(),
                         std::numeric_limits<double>::infinity(), damping) {}

  /// Constructor to create a revolute joint between two bodies so that
  /// frame Jp attached to the parent body P and frame Jc attached to the child
  /// body C, rotate relatively to one another about a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and rotation angle.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of revolution for this joint. Given
  ///   that frame Jc only rotates with respect to Jp and their origins are
  ///   coincident at all times, the components of `axis` in either frame Jp or
  ///   Jc are exactly the same, that is, `axis_Jp = axis_Jc`. In other words,
  ///   `axis_Jp` (or `axis_Jc`) is the eigenvector of `R_JpJc` with eigenvalue
  ///   equal to one. This vector can have any length, only the direction is
  ///   used.
  /// @param[in] pos_lower_limit
  ///   Lower position limit, in radians, for the rotation coordinate
  ///   (see get_angle()).
  /// @param[in] pos_upper_limit
  ///   Upper position limit, in radians, for the rotation coordinate
  ///   (see get_angle()).
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e.
  ///   opposing motion, with ω the angular rate for `this` joint (see
  ///   get_angular_rate()).
  /// @throws std::exception if the L2 norm of `axis` is less than the square
  /// root of machine epsilon.
  /// @throws std::exception if damping is negative.
  /// @throws std::exception if pos_lower_limit > pos_upper_limit.
  RevoluteJoint(const std::string& name, const Frame<T>& frame_on_parent,
                const Frame<T>& frame_on_child, const Vector3<double>& axis,
                double pos_lower_limit, double pos_upper_limit,
                double damping = 0);

  ~RevoluteJoint() final;

  const std::string& type_name() const final;

  /// Returns the axis of revolution of `this` joint as a unit vector.
  /// Since the measures of this axis in either frame F or M are the same (see
  /// this class's documentation for frame definitions) then,
  /// `axis = axis_Jp = axis_Jc`.
  const Vector3<double>& revolute_axis() const { return axis_; }

  /// Returns `this` joint's default damping constant in N⋅m⋅s.
  double default_damping() const { return this->default_damping_vector()[0]; }

  /// Sets the default value of viscous damping for this joint, in N⋅m⋅s.
  /// @throws std::exception if damping is negative.
  /// @throws std::exception if this element is not associated with a
  ///   MultibodyPlant.
  /// @pre the MultibodyPlant must not be finalized.
  void set_default_damping(double damping) {
    DRAKE_THROW_UNLESS(damping >= 0);
    DRAKE_THROW_UNLESS(this->has_parent_tree());
    DRAKE_DEMAND(!this->get_parent_tree().is_finalized());
    this->set_default_damping_vector(Vector1d(damping));
  }

  /// Returns the position lower limit for `this` joint in radians.
  double position_lower_limit() const {
    return this->position_lower_limits()[0];
  }

  /// Returns the position upper limit for `this` joint in radians.
  double position_upper_limit() const {
    return this->position_upper_limits()[0];
  }

  /// Returns the velocity lower limit for `this` joint in radians / s.
  double velocity_lower_limit() const {
    return this->velocity_lower_limits()[0];
  }

  /// Returns the velocity upper limit for `this` joint in radians / s.
  double velocity_upper_limit() const {
    return this->velocity_upper_limits()[0];
  }

  /// Returns the acceleration lower limit for `this` joint in radians / s².
  double acceleration_lower_limit() const {
    return this->acceleration_lower_limits()[0];
  }

  /// Returns the acceleration upper limit for `this` joint in radians / s².
  double acceleration_upper_limit() const {
    return this->acceleration_upper_limits()[0];
  }

  /// @name Context-dependent value access
  /// @{

  /// Gets the rotation angle of `this` mobilizer from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The angle coordinate of `this` joint stored in the `context`.
  const T& get_angle(const Context<T>& context) const {
    return get_mobilizer().get_angle(context);
  }

  /// Sets the `context` so that the generalized coordinate corresponding to the
  /// rotation angle of `this` joint equals `angle`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] angle
  ///   The desired angle in radians to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const RevoluteJoint<T>& set_angle(Context<T>* context, const T& angle) const {
    get_mobilizer().SetAngle(context, angle);
    return *this;
  }

  void set_random_angle_distribution(const symbolic::Expression& angle) {
    get_mutable_mobilizer().set_random_position_distribution(
        Vector1<symbolic::Expression>{angle});
  }

  /// Gets the rate of change, in radians per second, of `this` joint's
  /// angle (see get_angle()) from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The rate of change of `this` joint's angle as stored in the
  /// `context`.
  const T& get_angular_rate(const Context<T>& context) const {
    return get_mobilizer().get_angular_rate(context);
  }

  /// Sets the rate of change, in radians per second, of this `this` joint's
  /// angle to `angle`. The new rate of change `angle` gets stored in
  /// `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] angle
  ///   The desired rate of change of `this` joints's angle in radians per
  ///   second. (Should have been named `rate` or `angular_rate`.)
  /// @returns a constant reference to `this` joint.
  const RevoluteJoint<T>& set_angular_rate(Context<T>* context,
                                           const T& angle) const {
    get_mobilizer().SetAngularRate(context, angle);
    return *this;
  }

  /// Returns the Context dependent damping coefficient stored as a parameter in
  /// `context`. Refer to default_damping() for details.
  /// @param[in] context The context storing the state and parameters for the
  /// model to which `this` joint belongs.
  const T& GetDamping(const Context<T>& context) const {
    return this->GetDampingVector(context)[0];
  }

  /// Sets the value of the viscous damping coefficient for this joint, stored
  /// as a parameter in `context`. Refer to default_damping() for details.
  /// @param[out] context The context storing the state and parameters for the
  /// model to which `this` joint belongs.
  /// @param[in] damping The damping value.
  /// @throws std::exception if `damping` is negative.
  void SetDamping(Context<T>* context, const T& damping) const {
    DRAKE_THROW_UNLESS(damping >= 0);
    this->SetDampingVector(context, Vector1<T>(damping));
  }

  /// @}

  /// Gets the default rotation angle. Wrapper for the more general
  /// `Joint::default_positions()`.
  /// @returns The default angle of `this` stored in `default_positions_`
  double get_default_angle() const { return this->default_positions()[0]; }

  /// Sets the `default_positions` of this joint (in this case a single angle).
  /// @param[in] angle
  ///   The desired default angle of the joint
  void set_default_angle(double angle) {
    this->set_default_positions(Vector1d{angle});
  }

  /// Adds into `forces` a given `torque` for `this` joint that is to be applied
  /// about the joint's axis. The torque is defined to be positive according to
  /// the right-hand-rule with the thumb aligned in the direction of `this`
  /// joint's axis. That is, a positive torque causes a positive rotational
  /// acceleration according to the right-hand-rule around the joint's axis.
  ///
  /// @note A torque is the moment of a set of forces whose resultant is zero.
  void AddInTorque(const systems::Context<T>& context, const T& torque,
                   MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(this->has_parent_tree());
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
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const final {
    // Right now we assume all the forces in joint_tau go into a single
    // mobilizer.
    DRAKE_DEMAND(joint_dof == 0);
    Eigen::Ref<VectorX<T>> tau_mob =
        get_mobilizer().get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  /// Joint<T> override called through public NVI, Joint::AddInDamping().
  /// Therefore arguments were already checked to be valid.
  /// This method adds into `forces` a dissipative torque according to the
  /// viscous law `τ = -d⋅ω`, with d the damping coefficient (see
  /// default_damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final {
    const T damping_torque =
        -this->GetDamping(context) * get_angular_rate(context);
    AddInTorque(context, damping_torque, forces);
  }

 private:
  int do_get_velocity_start() const final {
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 1; }

  int do_get_position_start() const final {
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const final { return 1; }

  std::string do_get_position_suffix(int index) const final {
    return get_mobilizer().position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const final {
    return get_mobilizer().velocity_suffix(index);
  }

  void do_set_default_positions(
      const VectorX<double>& default_positions) final {
    if (this->has_mobilizer()) {
      get_mutable_mobilizer().set_default_position(default_positions);
    }
  }

  const T& DoGetOnePosition(const systems::Context<T>& context) const final {
    return get_angle(context);
  }

  const T& DoGetOneVelocity(const systems::Context<T>& context) const final {
    return get_angular_rate(context);
  }

  // Joint<T> overrides:
  std::unique_ptr<internal::Mobilizer<T>> MakeMobilizerForJoint(
      const internal::SpanningForest::Mobod& mobod,
      internal::MultibodyTree<T>* tree) const final;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const final;

  std::unique_ptr<Joint<T>> DoShallowClone() const final;

  // Make RevoluteJoint templated on every other scalar type a friend of
  // RevoluteJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of RevoluteJoint<T>.
  template <typename>
  friend class RevoluteJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::RevoluteMobilizer<T>& get_mobilizer() const {
    return this->template get_mobilizer_downcast<internal::RevoluteMobilizer>();
  }

  internal::RevoluteMobilizer<T>& get_mutable_mobilizer() {
    return this->template get_mutable_mobilizer_downcast<
        internal::RevoluteMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  Vector3<double> axis_;
};

template <typename T>
const char RevoluteJoint<T>::kTypeName[] = "revolute";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RevoluteJoint);
