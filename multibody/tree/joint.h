#pragma once

#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/fixed_offset_frame.h"
#include "drake/multibody/tree/mobilizer.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"
#include "drake/multibody/tree/rigid_body.h"
#include "drake/systems/framework/context.h"

namespace drake {
namespace multibody {

namespace internal {
// This is a class used by MultibodyTree internals to create the implementation
// for a particular joint object.
template <typename T>
class JointImplementationBuilder;
}  // namespace internal

/// A %Joint models the kinematical relationship which characterizes the
/// possible relative motion between two bodies.
/// The two bodies connected by this %Joint object are referred to as _parent_
/// and _child_ bodies. The parent/child ordering defines the sign conventions
/// for the generalized coordinates and the coordinate ordering for multi-DOF
/// joints.
/// A %Joint is a model of a physical kinematic constraint between two bodies,
/// a constraint that in the real physical system does not specify a tree
/// ordering.
/// @image html drake/multibody/plant/images/BodyParentChildJoint.png width=50%
///
/// In Drake we define a frame F rigidly attached to the parent body P with pose
/// `X_PF` and a frame M rigidly attached to the child body B with pose `X_BM`.
/// A %Joint object specifies a kinematic relation between frames F and M,
/// which in turn imposes a kinematic relation between bodies P and B.
///
/// Typical joints include the ball joint, to allow unrestricted rotations about
/// a given point, the revolute joint, that constraints two bodies to rotate
/// about a given common axis, etc.
///
/// Consider the following example to build a simple pendulum system:
///
/// @code
/// MultibodyPlant<double> plant(0.0);
/// // ... Code here to setup quantities below as mass, com, etc. ...
/// const RigidBody<double>& pendulum =
///   plant.AddRigidBody(SpatialInertia<double>(mass, com, unit_inertia));
/// // We will connect the pendulum body to the world using a RevoluteJoint.
/// // In this simple case the parent body P is the model's world body and frame
/// // F IS the world frame.
/// // Additionally, we need to specify the pose of frame M on the pendulum's
/// // body frame B.
/// // Say we declared and initialized X_BM...
/// const RevoluteJoint<double>& elbow =
///   plant.AddJoint<RevoluteJoint>(
///     "Elbow",                /* joint name */
///     plant.world_body(),     /* parent body */
///     {},                     /* frame F IS the world frame W */
///     pendulum,               /* child body, the pendulum */
///     X_BM,                   /* pose of frame M in the body frame B */
///     Vector3d::UnitZ());     /* revolute axis in this case */
/// @endcode
///
/// @warning Do not ever attempt to instantiate and manipulate %Joint objects
/// on the stack; it will fail. Add joints to your plant using the provided API
/// MultibodyPlant::AddJoint() as in the example above.
///
/// @tparam_default_scalar
template <typename T>
class Joint : public MultibodyElement<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Joint)

  /// Creates a joint between two Frame objects which imposes a given kinematic
  /// relation between frame F attached on the parent body P and frame M
  /// attached on the child body B. The joint will be initialized to the model
  /// instance from @p frame_on_child (this is the typical convention for joints
  /// between the world and a model, or between two models (e.g. an arm to a
  /// gripper)).  See this class's documentation for further details.
  ///
  /// @param[in] name
  ///   A string with a name identifying `this` joint.
  /// @param[in] frame_on_parent
  ///   The frame F attached on the parent body connected by this joint.
  /// @param[in] frame_on_child
  ///   The frame M attached on the child body connected by this joint.
  /// @param[in] damping
  ///   A vector of viscous damping coefficients, of size num_velocities().
  ///   See default_damping_vector() for details.
  /// @param[in] pos_lower_limits
  ///   A vector storing the lower limit for each generalized position.
  ///   It must have the same size as `pos_upper_limit`.
  ///   A value equal to -∞ implies no lower limit.
  /// @param[in] pos_upper_limits
  ///   A vector storing the upper limit for each generalized position.
  ///   It must have the same size as `pos_lower_limit`.
  ///   A value equal to +∞ implies no upper limit.
  /// @param[in] vel_lower_limits
  ///   A vector storing the lower limit for each generalized velocity.
  ///   It must have the same size as `vel_upper_limit`.
  ///   A value equal to -∞ implies no lower limit.
  /// @param[in] vel_upper_limits
  ///   A vector storing the upper limit for each generalized velocity.
  ///   It must have the same size as `vel_lower_limit`.
  ///   A value equal to +∞ implies no upper limit.
  /// @param[in] acc_lower_limits
  ///   A vector storing the lower limit for each generalized acceleration.
  ///   It must have the same size as `acc_upper_limit`.
  ///   A value equal to -∞ implies no lower limit.
  /// @param[in] acc_upper_limits
  ///   A vector storing the upper limit for each generalized acceleration.
  ///   It must have the same size as `acc_lower_limit`.
  ///   A value equal to +∞ implies no upper limit.
  Joint(const std::string& name, const Frame<T>& frame_on_parent,
        const Frame<T>& frame_on_child, VectorX<double> damping,
        const VectorX<double>& pos_lower_limits,
        const VectorX<double>& pos_upper_limits,
        const VectorX<double>& vel_lower_limits,
        const VectorX<double>& vel_upper_limits,
        const VectorX<double>& acc_lower_limits,
        const VectorX<double>& acc_upper_limits)
      : MultibodyElement<T>(frame_on_child.model_instance()),
        name_(name),
        frame_on_parent_(frame_on_parent),
        frame_on_child_(frame_on_child),
        damping_(std::move(damping)),
        pos_lower_limits_(pos_lower_limits),
        pos_upper_limits_(pos_upper_limits),
        vel_lower_limits_(vel_lower_limits),
        vel_upper_limits_(vel_upper_limits),
        acc_lower_limits_(acc_lower_limits),
        acc_upper_limits_(acc_upper_limits) {
    // TODO(Mitiguy) Per discussion in PR# 13961 and issues #12789 and #13040,
    //  consider changing frame F to frame Jp and changing frame M to frame Jc.
    // Notice `this` joint references `frame_on_parent` and `frame_on_child` and
    // therefore they must outlive it.
    DRAKE_DEMAND(pos_lower_limits.size() == pos_upper_limits.size());
    DRAKE_DEMAND(vel_lower_limits.size() == vel_upper_limits.size());
    DRAKE_DEMAND(acc_lower_limits.size() == acc_upper_limits.size());
    DRAKE_DEMAND(damping_.size() == vel_lower_limits.size());

    // Verify that lower_limit <= upper_limit, elementwise.
    DRAKE_DEMAND((pos_lower_limits.array() <= pos_upper_limits.array()).all());
    DRAKE_DEMAND((vel_lower_limits.array() <= vel_upper_limits.array()).all());
    DRAKE_DEMAND((acc_lower_limits.array() <= acc_upper_limits.array()).all());

    // N.B. We cannot use `num_positions()` here because it is virtual.
    const int num_positions = pos_lower_limits.size();

    // initialize the default positions.
    default_positions_ = VectorX<double>::Zero(num_positions);
  }

  /// Additional constructor overload for joints with zero damping. Refer to
  /// the more general constructor signature taking damping for further details
  /// on the rest of the arguments for this constructor.
  Joint(const std::string& name, const Frame<T>& frame_on_parent,
        const Frame<T>& frame_on_child, const VectorX<double>& pos_lower_limits,
        const VectorX<double>& pos_upper_limits,
        const VectorX<double>& vel_lower_limits,
        const VectorX<double>& vel_upper_limits,
        const VectorX<double>& acc_lower_limits,
        const VectorX<double>& acc_upper_limits)
      : Joint(name, frame_on_parent, frame_on_child,
              VectorX<double>::Zero(vel_lower_limits.size()), pos_lower_limits,
              pos_upper_limits, vel_lower_limits, vel_upper_limits,
              acc_lower_limits, acc_upper_limits) {}

  virtual ~Joint() {}

  /// Returns this element's unique index.
  JointIndex index() const { return this->template index_impl<JointIndex>(); }

  /// Returns this element's unique ordinal. The joint's ordinal is a unique
  /// index into contiguous containers that have an entry for each Joint, such
  /// as the vector valued reaction forces (see
  /// MultibodyPlant::get_reaction_forces_output_port()). The ordinal value will
  /// be updated (if needed) when joints are removed from the parent plant so
  /// that the set of ordinal values is a bijection with [0, num_joints()).
  /// Ordinals are assigned in the order that joints are added to the plant,
  /// thus a set of joints sorted by ordinal has the same ordering as if it were
  /// sorted by JointIndex. If joints have been removed from the plant, do *not*
  /// use index() to access contiguous containers with entries per Joint.
  int ordinal() const { return this->ordinal_impl(); }

  /// Returns the name of this joint.
  const std::string& name() const { return name_; }

  /// Returns a const reference to the parent body P.
  const RigidBody<T>& parent_body() const {
    return frame_on_parent_.body();
  }

  /// Returns a const reference to the child body B.
  const RigidBody<T>& child_body() const {
    return frame_on_child_.body();
  }

  /// Returns a const reference to the frame F attached on the parent body P.
  const Frame<T>& frame_on_parent() const {
    return frame_on_parent_;
  }

  /// Returns a const reference to the frame M attached on the child body B.
  const Frame<T>& frame_on_child() const {
    return frame_on_child_;
  }

  /// Returns a string identifying the type of `this` joint, such as "revolute"
  /// or "prismatic".
  virtual const std::string& type_name() const = 0;

  /// Returns the index to the first generalized velocity for this joint
  /// within the vector v of generalized velocities for the full multibody
  /// system.
  int velocity_start() const {
    return do_get_velocity_start();
  }

  /// Returns the number of generalized velocities describing this joint.
  int num_velocities() const {
    DRAKE_ASSERT(0 <= do_get_num_velocities() && do_get_num_velocities() <= 6);
    return do_get_num_velocities();
  }

  /// Returns the index to the first generalized position for this joint
  /// within the vector q of generalized positions for the full multibody
  /// system.
  int position_start() const {
    return do_get_position_start();
  }

  /// Returns the number of generalized positions describing this joint.
  int num_positions() const {
    DRAKE_ASSERT(0 <= do_get_num_positions() && do_get_num_positions() <= 7);
    return do_get_num_positions();
  }

  /// Returns true if this joint's mobility allows relative rotation of the
  /// two frames associated with this joint.
  /// @pre the MultibodyPlant must be finalized.
  /// @see can_translate()
  bool can_rotate() const;

  /// Returns true if this joint's mobility allows relative translation of the
  /// two frames associated with this joint.
  /// @pre the MultibodyPlant must be finalized.
  /// @see can_rotate()
  bool can_translate() const;

  /// Returns a string suffix (e.g. to be appended to the name()) to identify
  /// the `k`th position in this joint.  @p position_index_in_joint must be
  /// in [0, num_positions()).
  /// @pre the MultibodyPlant must be finalized.
  std::string position_suffix(int position_index_in_joint) const {
    DRAKE_DEMAND(0 <= position_index_in_joint &&
                 position_index_in_joint < num_positions());
    DRAKE_DEMAND(has_implementation());
    return do_get_position_suffix(position_index_in_joint);
  }

  /// Returns a string suffix (e.g. to be appended to the name()) to identify
  /// the `k`th velocity in this joint. @p velocity_index_in_joint must be
  /// in [0, num_velocities()).
  /// @pre the MultibodyPlant must be finalized.
  std::string velocity_suffix(int velocity_index_in_joint) const {
    DRAKE_DEMAND(0 <= velocity_index_in_joint &&
                 velocity_index_in_joint < num_velocities());
    DRAKE_DEMAND(has_implementation());
    return do_get_velocity_suffix(velocity_index_in_joint);
  }

  /// Returns the position coordinate for joints with a single degree of
  /// freedom.
  /// @throws std::exception if the joint does not have a single degree of
  /// freedom.
  const T& GetOnePosition(const systems::Context<T>& context) const {
    DRAKE_THROW_UNLESS(num_positions() == 1);
    return DoGetOnePosition(context);
  }

  /// Returns the velocity coordinate for joints with a single degree of
  /// freedom.
  /// @throws std::exception if the joint does not have a single degree of
  /// freedom.
  const T& GetOneVelocity(const systems::Context<T>& context) const {
    DRAKE_THROW_UNLESS(num_velocities() == 1);
    return DoGetOneVelocity(context);
  }

  /// Adds into `forces` a force along the one of the joint's degrees of
  /// freedom indicated by index `joint_dof`.
  /// The meaning for this degree of freedom and even its dimensional units
  /// depend on the specific joint sub-class. For a RevoluteJoint for instance,
  /// `joint_dof` can only be 0 since revolute joints's motion subspace only has
  /// one degree of freedom, while the units of `joint_tau` are those of torque
  /// (N⋅m in the MKS system of units). For multi-dof joints please refer to
  /// the documentation provided by specific joint sub-classes regarding the
  /// meaning of `joint_dof`.
  ///
  /// @param[in] context
  ///   The context storing the state and parameters for the model to which
  ///   `this` joint belongs.
  /// @param[in] joint_dof
  ///   Index specifying one of the degrees of freedom for this joint. The index
  ///   must be in the range `0 <= joint_dof < num_velocities()` or otherwise
  ///   this method will abort.
  /// @param[in] joint_tau
  ///   Generalized force corresponding to the degree of freedom indicated by
  ///   `joint_dof` to be added into `forces`.
  /// @param[out] forces
  ///   On return, this method will add force `joint_tau` for the degree of
  ///   freedom `joint_dof` into the output `forces`. This method aborts if
  ///   `forces` is `nullptr` or if `forces` doest not have the right sizes to
  ///   accommodate a set of forces for the model to which this joint belongs.
  // NVI to DoAddInOneForce().
  void AddInOneForce(
      const systems::Context<T>& context,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(0 <= joint_dof && joint_dof < num_velocities());
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    DoAddInOneForce(context, joint_dof, joint_tau, forces);
  }

  /// Adds into `forces` the force due to damping within `this` joint.
  ///
  /// @param[in] context
  ///   The context storing the state and parameters for the model to which
  ///   `this` joint belongs.
  /// @param[out] forces
  ///   On return, this method will add the force due to damping within `this`
  ///   joint. This method aborts if `forces` is `nullptr` or if `forces` does
  ///   not have the right sizes to accommodate a set of forces for the model
  ///   to which this joint belongs.
  // NVI to DoAddInOneForce().
  void AddInDamping(
      const systems::Context<T>& context, MultibodyForces<T>* forces) const {
    DRAKE_DEMAND(forces != nullptr);
    DRAKE_DEMAND(forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    DoAddInDamping(context, forces);
  }

  /// Lock the joint. Its generalized velocities will be 0 until it is
  /// unlocked.
  void Lock(systems::Context<T>* context) const {
    DRAKE_DEMAND(implementation_->has_mobilizer());
    implementation_->mobilizer->Lock(context);
  }

  /// Unlock the joint.
  void Unlock(systems::Context<T>* context) const {
    DRAKE_DEMAND(implementation_->has_mobilizer());
    implementation_->mobilizer->Unlock(context);
  }

  /// @return true if the joint is locked, false otherwise.
  bool is_locked(const systems::Context<T>& context) const {
    DRAKE_DEMAND(implementation_->has_mobilizer());
    return implementation_->mobilizer->is_locked(context);
  }

  /// @name Methods to get and set the limits of `this` joint. For position
  /// limits, the layout is the same as the generalized position's. For
  /// velocity and acceleration limits, the layout is the same as the
  /// generalized velocity's. A limit with value +/- ∞ implies no upper or
  /// lower limit.
  /// @{
  /// Returns the position lower limits.
  const VectorX<double>& position_lower_limits() const {
    return pos_lower_limits_;
  }

  /// Returns the position upper limits.
  const VectorX<double>& position_upper_limits() const {
    return pos_upper_limits_;
  }

  /// Returns the velocity lower limits.
  const VectorX<double>& velocity_lower_limits() const {
    return vel_lower_limits_;
  }

  /// Returns the velocity upper limits.
  const VectorX<double>& velocity_upper_limits() const {
    return vel_upper_limits_;
  }

  /// Returns the acceleration lower limits.
  const VectorX<double>& acceleration_lower_limits() const {
    return acc_lower_limits_;
  }

  /// Returns the acceleration upper limits.
  const VectorX<double>& acceleration_upper_limits() const {
    return acc_upper_limits_;
  }

  /// Returns the default positions.
  const VectorX<double>& default_positions() const {
    return default_positions_;
  }

  /// Sets the position limits to @p lower_limits and @p upper_limits.
  /// @throws std::exception if the dimension of @p lower_limits or
  /// @p upper_limits does not match num_positions().
  /// @throws std::exception if any of @p lower_limits is larger than the
  /// corresponding term in @p upper_limits.
  /// @note Setting the position limits does not affect the
  /// `default_positions()`, regardless of whether the current
  /// `default_positions()` satisfy the new position limits.
  void set_position_limits(const VectorX<double>& lower_limits,
                           const VectorX<double>& upper_limits) {
    DRAKE_THROW_UNLESS(lower_limits.size() == upper_limits.size());
    DRAKE_THROW_UNLESS(lower_limits.size() == num_positions());
    DRAKE_THROW_UNLESS((lower_limits.array() <= upper_limits.array()).all());
    pos_lower_limits_ = lower_limits;
    pos_upper_limits_ = upper_limits;
  }

  /// Sets the velocity limits to @p lower_limits and @p upper_limits.
  /// @throws std::exception if the dimension of @p lower_limits or
  /// @p upper_limits does not match num_velocities().
  /// @throws std::exception if any of @p lower_limits is larger than the
  /// corresponding term in @p upper_limits.
  void set_velocity_limits(const VectorX<double>& lower_limits,
                           const VectorX<double>& upper_limits) {
    DRAKE_THROW_UNLESS(lower_limits.size() == upper_limits.size());
    DRAKE_THROW_UNLESS(lower_limits.size() == num_velocities());
    DRAKE_THROW_UNLESS((lower_limits.array() <= upper_limits.array()).all());
    vel_lower_limits_ = lower_limits;
    vel_upper_limits_ = upper_limits;
  }

  /// Sets the acceleration limits to @p lower_limits and @p upper_limits.
  /// @throws std::exception if the dimension of @p lower_limits or
  /// @p upper_limits does not match num_velocities().
  /// @throws std::exception if any of @p lower_limits is larger than the
  /// corresponding term in @p upper_limits.
  void set_acceleration_limits(const VectorX<double>& lower_limits,
                               const VectorX<double>& upper_limits) {
    DRAKE_THROW_UNLESS(lower_limits.size() == upper_limits.size());
    DRAKE_THROW_UNLESS(lower_limits.size() == num_velocities());
    DRAKE_THROW_UNLESS((lower_limits.array() <= upper_limits.array()).all());
    acc_lower_limits_ = lower_limits;
    acc_upper_limits_ = upper_limits;
  }

  /// Sets the default positions to @p default_positions. Joint subclasses are
  /// expected to implement the do_set_default_positions().
  /// @throws std::exception if the dimension of @p default_positions does not
  /// match num_positions().
  /// @note The values in @p default_positions are NOT constrained to be within
  /// `position_lower_limits()` and `position_upper_limits()`.
  void set_default_positions(const VectorX<double>& default_positions) {
    DRAKE_THROW_UNLESS(default_positions.size() == num_positions());
    default_positions_ = default_positions;
    do_set_default_positions(default_positions);
  }

  // TODO(sherm1) Consider implementing SetDefaultPose() for every joint type,
  //  with the joint responsible for making a "best effort" to match the pose if
  //  it can't do so exactly. Simbody has that feature and it has proven very
  //  useful in practice.

  /// Sets this %Joint's default generalized positions q₀ such that the pose
  /// of the child frame M in the parent frame F best matches the given pose.
  /// The pose is given by a RigidTransform `X_FM`, but a %Joint will
  /// represent pose differently.
  /// @note Currently this is implemented only for floating (6 dof) joints
  /// which can represent any pose.
  /// @throws std::exception if called for any %Joint type that does not
  /// implement this function.
  /// @see get_default_positions() to see the resulting q₀ after this call.
  /// @see SetDefaultPosePair() for an alternative using a quaternion
  void SetDefaultPose(const math::RigidTransform<double>& X_FM) {
    SetDefaultPosePair(X_FM.rotation().ToQuaternion(), X_FM.translation());
  }

  /// Returns this %Joint's default pose as a RigidTransform X_FM.
  /// @note Currently this is implemented only for floating (6 dof) joints
  /// which can represent any pose.
  /// @throws std::exception if called for any %Joint type that does not
  /// implement this function.
  /// @retval X_FM The default pose as a rigid transform.
  /// @see get_default_positions() to see the generalized positions q₀ that this
  ///      joint used to generate the returned transform.
  math::RigidTransform<double> GetDefaultPose() const {
    auto pose_pair = GetDefaultPosePair();
    return math::RigidTransform(pose_pair.first, pose_pair.second);
  }

  // BTW These are implemented with a (quaternion,vector) pair rather than a
  // rigid transform so that we can guarantee to preserve bit-perfect results
  // when mapping a floating body default pose to the default positions of its
  // inboard quaternion floating joint. Users should prefer the above versions.

  /// (Advanced) This is the same as SetDefaultPose() except it takes the
  /// pose as a (quaternion, translation vector) pair.
  /// @see SetDefaultPose() for more information
  void SetDefaultPosePair(const Quaternion<double>& q_FM,
                          const Vector3<double>& p_FM) {
    DoSetDefaultPosePair(q_FM, p_FM);
  }

  /// (Advanced) This is the same as GetDefaultPose() except it returns this
  /// %Joint's default pose as a (quaternion, translation vector) pair.
  /// @retval q_FM,p_FM The default pose as a (quaternion, translation) pair.
  /// @see GetDefaultPose() for more information
  std::pair<Eigen::Quaternion<double>, Vector3<double>> GetDefaultPosePair()
      const {
    return DoGetDefaultPosePair();
  }

  /// @}

  /// Returns all default damping coefficients for joints that model viscous
  /// damping, of size num_velocities(). Joints that do not model damping return
  /// a zero vector of size num_velocities(). If vj is the vector of generalized
  /// velocities for this joint, of size num_velocities(), viscous damping
  /// models a generalized force at the joint of the form tau = -diag(dj)⋅vj,
  /// with dj the vector returned by this function. The units of the
  /// coefficients will depend on the specific joint type. For instance, for a
  /// revolute joint where vj is an angular velocity with units of rad/s and tau
  /// having units of N⋅m, the coefficient of viscous damping has units of
  /// N⋅m⋅s. Refer to each joint's documentation for further details.
  const VectorX<double>& default_damping_vector() const { return damping_; }

  DRAKE_DEPRECATED("2024-06-01", "Use default_damping_vector() instead.")
  const VectorX<double>& damping_vector() const { return damping_; }

  /// Returns the Context dependent damping coefficients stored as parameters in
  /// `context`. Refer to default_damping_vector() for details.
  /// @param[in] context The context storing the state and parameters for the
  /// model to which `this` joint belongs.
  const VectorX<T>& GetDampingVector(const systems::Context<T>& context) const {
    return context.get_numeric_parameter(damping_parameter_index_).value();
  }

  /// Sets the default value of the viscous damping coefficients for this joint.
  /// Refer to default_damping_vector() for details.
  /// @throws std::exception if damping.size() != num_velocities().
  /// @throws std::exception if any of the damping coefficients is negative.
  /// @pre the MultibodyPlant must not be finalized.
  void set_default_damping_vector(const VectorX<double>& damping) {
    DRAKE_THROW_UNLESS(damping.size() == num_velocities());
    DRAKE_THROW_UNLESS((damping.array() >= 0).all());
    DRAKE_DEMAND(!this->get_parent_tree().topology_is_valid());
    damping_ = damping;
  }

  /// Sets the value of the viscous damping coefficients for this joint, stored
  /// as parameters in `context`. Refer to default_damping_vector() for details.
  /// @param[out] context The context storing the state and parameters for the
  /// model to which `this` joint belongs.
  /// @param[in] damping The vector of damping values.
  /// @throws std::exception if damping.size() != num_velocities().
  /// @throws std::exception if any of the damping coefficients is negative.
  /// @note Some multi-dof joints may have specific semantics for their damping
  /// vector that are not enforced here. For instance, QuaternionFloatingJoint
  /// assumes identical damping values for all 3 angular velocity components and
  /// identical damping values for all 3 translational velocity components. It
  /// will thus use `angular_damping = damping[0]` and `translational_damping =
  /// damping[3]`. Refer to the particular subclass for more semantic
  /// information.
  void SetDampingVector(systems::Context<T>* context,
                        const VectorX<T>& damping) const {
    DRAKE_THROW_UNLESS(damping.size() == num_velocities());
    DRAKE_THROW_UNLESS((damping.array() >= 0).all());
    context->get_mutable_numeric_parameter(damping_parameter_index_)
        .set_value(damping);
  }

  // Hide the following section from Doxygen.
#ifndef DRAKE_DOXYGEN_CXX
  // NVI to DoCloneToScalar() templated on the scalar type of the new clone to
  // be created. This method is intended to be called by
  // MultibodyTree::CloneToScalar().
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> CloneToScalar(
      internal::MultibodyTree<ToScalar>* tree_clone) const {
    std::unique_ptr<Joint<ToScalar>> joint_clone = DoCloneToScalar(*tree_clone);

    std::unique_ptr<typename Joint<ToScalar>::JointImplementation>
        implementation_clone =
        this->get_implementation().template CloneToScalar<ToScalar>(tree_clone);
    joint_clone->OwnImplementation(std::move(implementation_clone));

    return joint_clone;
  }

  const internal::Mobilizer<T>& GetMobilizerInUse() const {
    // Currently we model each joint with a mobilizer.
    DRAKE_DEMAND(get_implementation().has_mobilizer());
    return *get_implementation().mobilizer;
  }
#endif
  // End of hidden Doxygen section.

 protected:
  /// (Advanced) Structure containing all the information needed to build the
  /// MultibodyTree implementation for a %Joint. At MultibodyTree::Finalize() a
  /// %Joint creates a BluePrint of its implementation with MakeModelBlueprint()
  /// so that MultibodyTree can build an implementation for it.
  struct BluePrint {
    std::unique_ptr<internal::Mobilizer<T>> mobilizer;
    // TODO(sherm1): add constraints and force elements as needed.
  };

  /// (Advanced) A Joint is implemented in terms of MultibodyTree elements,
  /// typically a Mobilizer. However, some Joints may be better modeled with
  /// constraints or force elements. This object contains the internal details
  /// of the MultibodyTree implementation for a joint. The implementation does
  /// not own the MbT elements, it just keeps references to them. This is
  /// intentionally made a protected member so that derived classes have
  /// access to its definition.
  struct JointImplementation {
    /// Default constructor to create an empty implementation. Used by
    /// Joint::CloneToScalar().
    JointImplementation() {}

    /// This constructor creates an implementation for `this` joint from the
    /// blueprint provided.
    explicit JointImplementation(const BluePrint& blue_print) {
      DRAKE_DEMAND(blue_print.mobilizer != nullptr);
      mobilizer = blue_print.mobilizer.get();
    }

    /// Returns `true` if the implementation of this Joint uses a Mobilizer.
    bool has_mobilizer() const {
      return mobilizer != nullptr;
    }

    // Hide the following section from Doxygen.
#ifndef DRAKE_DOXYGEN_CXX
    // Helper method to be called within Joint::CloneToScalar() to clone its
    // implementation to the appropriate scalar type.
    template <typename ToScalar>
    std::unique_ptr<typename Joint<ToScalar>::JointImplementation>
    CloneToScalar(internal::MultibodyTree<ToScalar>* tree_clone) const {
      auto implementation_clone =
          std::make_unique<typename Joint<ToScalar>::JointImplementation>();
      internal::Mobilizer<ToScalar>* mobilizer_clone =
          &tree_clone->get_mutable_variant(*mobilizer);
      implementation_clone->mobilizer = mobilizer_clone;
      return implementation_clone;
    }
#endif
    // End of hidden Doxygen section.

    /// Reference (raw pointer) to the mobilizer implementing this Joint.
    internal::Mobilizer<T>* mobilizer{};
    // TODO(sherm1): add constraints and force elements as needed.
  };

  /// Implementation of the NVI velocity_start(), see velocity_start() for
  /// details. Note that this must be the offset within just the velocity
  /// vector, _not_ within the composite state vector.
  /// @note Implementations must meet the styleguide requirements for snake_case
  /// accessor methods.
  virtual int do_get_velocity_start() const = 0;

  /// Implementation of the NVI num_velocities(), see num_velocities() for
  /// details.
  /// @note Implementations must meet the styleguide requirements for snake_case
  /// accessor methods.
  virtual int do_get_num_velocities() const = 0;

  /// Implementation of the NVI position_start(), see position_start() for
  /// details.
  /// @note Implementations must meet the styleguide requirements for snake_case
  /// accessor methods.
  virtual int do_get_position_start() const = 0;

  /// Implementation of the NVI num_positions(), see num_positions() for
  /// details.
  /// @note Implementations must meet the styleguide requirements for
  /// snake_case accessor methods.
  virtual int do_get_num_positions() const = 0;

  /// Implementation of the NVI position_suffix(), see position_suffix() for
  /// details.  The suffix should contain only alphanumeric characters (e.g.
  /// 'wx' not '_wx' or '.wx').
  virtual std::string do_get_position_suffix(int index) const = 0;

  /// Implementation of the NVI velocity_suffix(), see velocity_suffix() for
  /// details.  The suffix should contain only alphanumeric characters (e.g.
  /// 'wx' not '_wx' or '.wx').
  virtual std::string do_get_velocity_suffix(int index) const = 0;

  /// Implementation of the NVI set_default_positions(), see
  /// set_default_positions() for details. It is the responsibility of the
  /// subclass to ensure that its joint implementation (i.e., mobilizer), should
  /// it have one, is updated with @p default_positions. Note that the
  /// %Joint base class also stores default_positions (as a VectorX); the
  /// implementing mobilizer should have the same value but as a fixed-size
  /// vector.
  /// @note Implementations must meet the styleguide requirements for snake_case
  /// accessor methods.
  virtual void do_set_default_positions(
      const VectorX<double>& default_positions) = 0;

  /// Implementation of the NVI SetDefaultPose(). This is optional for %Joint
  /// subclasses _except_ for floating (6 dof) Joints. The subclass
  /// should convert the input to the closest equivalent in generalized
  /// coordinates and invoke set_default_positions() to record them. If the
  /// subclass already uses (quaternion, translation) as generalized coordinates
  /// (i.e. it's a quaternion_floating_joint) it must store those exactly.
  virtual void DoSetDefaultPosePair(const Quaternion<double>& q_FM,
                                    const Vector3<double>& p_FM) {
    unused(q_FM, p_FM);
    throw std::logic_error(fmt::format(
        "SetDefaultPose(): not implemented for joint type {}.", type_name()));
  }

  /// Implementation of the NVI GetDefaultPose(). This is optional for %Joint
  /// subclasses _except_ for floating (6 dof) Joints. The subclass should
  /// convert its default_positions to pose X_FM and return that as a
  /// (quaternion, translation) pair. If the subclass already uses
  /// (quaternion, translation) as generalized coordinates (i.e. it's a
  /// quaternion_floating_joint) it must return those exactly (don't
  /// convert to a transform first).
  virtual std::pair<Eigen::Quaternion<double>, Vector3<double>>
  DoGetDefaultPosePair() const {
    throw std::logic_error(fmt::format(
        "GetDefaultPose(): not implemented for joint type {}.", type_name()));
  }

  /// Implementation of the NVI GetOnePosition() that must only be implemented
  /// by those joint subclasses that have a single degree of freedom.
  /// The default implementation for all other joints is to abort with an
  /// appropriate message.
  /// Revolute and prismatic are examples of joints that will want to implement
  /// this method.
  virtual const T& DoGetOnePosition(const systems::Context<T>&) const {
    throw std::domain_error(
        "GetOnePosition can only be called on single-dof joints.");
  }

  /// Implementation of the NVI GetOneVelocity() that must only be implemented
  /// by those joint subclasses that have a single degree of freedom.
  /// The default implementation for all other joints is to abort with an
  /// appropriate message.
  /// Revolute and prismatic are examples of joints that will want to implement
  /// this method.
  virtual const T& DoGetOneVelocity(const systems::Context<T>&) const {
    throw std::domain_error(
        "GetOneVelocity can only be called on single-dof joints.");
  }

  /// Adds into `forces` a force along the one of the joint's degrees of
  /// freedom given by `joint_dof`.
  /// How forces are added to a MultibodyTree model depends on the underlying
  /// implementation of a particular joint and therefore specific %Joint
  /// subclasses must provide a definition for this method. For instance, a
  /// revolute joint could be modeled with a single generalized coordinate for
  /// the angular rotation (implemented through a RevoluteMobilizer) or it could
  /// be modeled using a constraint that only allows rotation about the joint's
  /// axis but that constrains the motion in the other five degrees of freedom.
  /// This method is only called by the public NVI AddInOneForce() and therefore
  /// input arguments were checked to be valid.
  /// @see The public NVI AddInOneForce() for details.
  virtual void DoAddInOneForce(
      const systems::Context<T>& context,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const = 0;

  /// Adds into MultibodyForces the forces due to damping within `this` joint.
  /// How forces are added to a MultibodyTree model depends on the underlying
  /// implementation of a particular joint (for instance, mobilizer vs.
  /// constraint) and therefore specific %Joint subclasses must provide a
  /// definition for this method.
  /// The default implementation is a no-op for joints with no damping.
  virtual void DoAddInDamping(
      const systems::Context<T>&, MultibodyForces<T>*) const {}

  // Implements MultibodyElement::DoSetTopology(). Joints have no topology
  // though we could require them to have one in the future.
  void DoSetTopology(const internal::MultibodyTreeTopology&) override {}

  /// @name Methods to make a clone templated on different scalar types.
  /// @{
  /// Clones this %Joint (templated on T) to a joint templated on `double`.
  virtual std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const = 0;

  /// Clones this %Joint (templated on T) to a joint templated on AutoDiffXd.
  virtual std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const = 0;

  virtual std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const = 0;
  /// @}

  /// This method must be implemented by derived classes in order to provide
  /// JointImplementationBuilder a BluePrint of their internal implementation
  /// JointImplementation.
  virtual std::unique_ptr<BluePrint> MakeImplementationBlueprint() const = 0;

  /// Returns a const reference to the internal implementation of `this` joint.
  /// @warning The MultibodyTree model must have already been finalized, or
  /// this method will abort.
  const JointImplementation& get_implementation() const {
    // The MultibodyTree must have been finalized for the implementation to be
    // valid.
    DRAKE_DEMAND(this->get_parent_tree().topology_is_valid());
    return *implementation_;
  }

  /// Returns whether `this` joint owns a particular implementation.
  /// If the MultibodyTree has been finalized, this will return true.
  bool has_implementation() const { return implementation_ != nullptr; }

 private:
  // Make all other Joint<U> objects a friend of Joint<T> so they can make
  // Joint<ToScalar>::JointImplementation from CloneToScalar<ToScalar>().
  template <typename>
  friend class Joint;

  // JointImplementationBuilder is a friend so that it can access the
  // Joint<T>::BluePrint and protected method MakeImplementationBlueprint().
  friend class internal::JointImplementationBuilder<T>;

  // When an implementation is created, either by
  // internal::JointImplementationBuilder or by Joint::CloneToScalar(), this
  // method is called to pass ownership of an implementation to the Joint.
  void OwnImplementation(std::unique_ptr<JointImplementation> implementation) {
    implementation_ = std::move(implementation);
  }

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) final {
    // Declare a parameter for damping.
    damping_parameter_index_ = this->DeclareNumericParameter(
        tree_system, systems::BasicVector<T>(damping_.size()));
  }

  // Implementation for MultibodyElement::DoSetDefaultParameters().
  void DoSetDefaultParameters(systems::Parameters<T>* parameters) const final {
    // Set default damping.
    systems::BasicVector<T>& damping_parameter =
        parameters->get_mutable_numeric_parameter(damping_parameter_index_);
    damping_parameter.set_value(VectorX<T>(damping_));
  }

  std::string name_;
  const Frame<T>& frame_on_parent_;
  const Frame<T>& frame_on_child_;

  VectorX<double> damping_;

  // Joint position limits. These vectors have zero size for joints with no
  // such limits.
  VectorX<double> pos_lower_limits_;
  VectorX<double> pos_upper_limits_;

  // Joint velocity limits. These vectors have zero size for joints with no
  // such limits.
  VectorX<double> vel_lower_limits_;
  VectorX<double> vel_upper_limits_;

  // Joint acceleration limits. These vectors have zero size for joints with no
  // such limits.
  VectorX<double> acc_lower_limits_;
  VectorX<double> acc_upper_limits_;

  // Joint default position. This vector has zero size for joints with no state.
  VectorX<double> default_positions_;

  // The Joint<T> implementation:
  std::unique_ptr<JointImplementation> implementation_;

  // System parameter indices.
  systems::NumericParameterIndex damping_parameter_index_;
};

}  // namespace multibody
}  // namespace drake
