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
/// const Body<double>& pendulum =
///   plant.AddBody<RigidBody>(SpatialInertia<double>(mass, com, unit_inertia));
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
class Joint : public MultibodyElement<Joint, T, JointIndex> {
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
        const Frame<T>& frame_on_child, const VectorX<double>& pos_lower_limits,
        const VectorX<double>& pos_upper_limits,
        const VectorX<double>& vel_lower_limits,
        const VectorX<double>& vel_upper_limits,
        const VectorX<double>& acc_lower_limits,
        const VectorX<double>& acc_upper_limits)
      : MultibodyElement<Joint, T, JointIndex>(
        frame_on_child.model_instance()),
        name_(name),
        frame_on_parent_(frame_on_parent),
        frame_on_child_(frame_on_child),
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
    // Verify that lower_limit <= upper_limit, elementwise.
    DRAKE_DEMAND((pos_lower_limits.array() <= pos_upper_limits.array()).all());

    DRAKE_DEMAND(vel_lower_limits.size() == vel_upper_limits.size());
    DRAKE_DEMAND((vel_lower_limits.array() <= vel_upper_limits.array()).all());

    DRAKE_DEMAND(acc_lower_limits.size() == acc_upper_limits.size());
    DRAKE_DEMAND((acc_lower_limits.array() <= acc_upper_limits.array()).all());

    // N.B. We cannot use `num_positions()` here because it is virtual.
    const int num_positions = pos_lower_limits.size();

    // intialize the default positions.
    default_positions_ = VectorX<double>::Zero(num_positions);
  }

  virtual ~Joint() {}

  /// Returns the name of this joint.
  const std::string& name() const { return name_; }

  /// Returns a const reference to the parent body P.
  const Body<T>& parent_body() const {
    return frame_on_parent_.body();
  }

  /// Returns a const reference to the child body B.
  const Body<T>& child_body() const {
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
  /// unlocked. Locking is not yet supported for continuous-mode systems.
  /// @throws std::exception if the parent model uses continuous state.
  void Lock(systems::Context<T>* context) const {
    // Joint locking is only supported for discrete mode.
    // TODO(sherm1): extend the design to support continuous-mode systems.
    DRAKE_THROW_UNLESS(this->get_parent_tree().is_state_discrete());
    context->get_mutable_abstract_parameter(is_locked_parameter_index_)
        .set_value(true);
    this->get_parent_tree().GetMutableVelocities(context).segment(
        this->velocity_start(),
        this->num_velocities()).setZero();
  }

  /// Unlock the joint. Unlocking is not yet supported for continuous-mode
  /// systems.
  /// @throws std::exception if the parent model uses continuous state.
  void Unlock(systems::Context<T>* context) const {
    // Joint locking is only supported for discrete mode.
    // TODO(sherm1): extend the design to support continuous-mode systems.
    DRAKE_THROW_UNLESS(this->get_parent_tree().is_state_discrete());
    context->get_mutable_abstract_parameter(is_locked_parameter_index_)
        .set_value(false);
  }

  /// @return true if the joint is locked, false otherwise.
  bool is_locked(const systems::Context<T>& context) const {
    return context.get_parameters().template get_abstract_parameter<bool>(
        is_locked_parameter_index_);
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
  /// @}

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
#endif
  // End of hidden Doxygen section.

 protected:
  /// (Advanced) Structure containing all the information needed to build the
  /// MultibodyTree implementation for a %Joint. At MultibodyTree::Finalize() a
  /// %Joint creates a BluePrint of its implementation with MakeModelBlueprint()
  /// so that MultibodyTree can build an implementation for it.
  struct BluePrint {
    std::vector<std::unique_ptr<internal::Mobilizer<T>>> mobilizers_;
    // TODO(amcastro-tri): add force elements, constraints, bodies.
  };

  /// (Advanced) A Joint is implemented in terms of MultibodyTree elements such
  /// as bodies, mobilizers, force elements and constraints. This object
  /// contains the internal details of the MultibodyTree implementation for a
  /// joint. The implementation does not own the MBT elements, it just keeps
  /// references to them.
  /// This is intentionally made a protected member so that derived classes have
  /// access to its definition.
  struct JointImplementation {
    /// Default constructor to create an empty implementation. Used by
    /// Joint::CloneToScalar().
    JointImplementation() {}

    /// This constructor creates an implementation for `this` joint from the
    /// blueprint provided.
    explicit JointImplementation(const BluePrint& blue_print) {
      DRAKE_DEMAND(static_cast<int>(blue_print.mobilizers_.size()) != 0);
      for (const auto& mobilizer : blue_print.mobilizers_) {
        mobilizers_.push_back(mobilizer.get());
      }
    }

    /// Returns the number of mobilizers in this implementation.
    int num_mobilizers() const {
      return static_cast<int>(mobilizers_.size());
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
      for (const internal::Mobilizer<T>* mobilizer : mobilizers_) {
        internal::Mobilizer<ToScalar>* mobilizer_clone =
            &tree_clone->get_mutable_variant(*mobilizer);
        implementation_clone->mobilizers_.push_back(mobilizer_clone);
      }
      return implementation_clone;
    }
#endif
    // End of hidden Doxygen section.

    /// References (raw pointers) to the mobilizers that make part of this
    /// implementation.
    std::vector<internal::Mobilizer<T>*> mobilizers_;
    // TODO(amcastro-tri): add force elements, constraints, bodies, etc.
  };

  /// Implementation of the NVI velocity_start(), see velocity_start() for
  /// details.
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
  /// subclass to ensure that their joint implementation, should they have one,
  /// is updated with @p default_positions.
  /// @note Implementations must meet the styleguide requirements for snake_case
  /// accessor methods.
  virtual void do_set_default_positions(
      const VectorX<double>& default_positions) = 0;

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

  // Implementation for MultibodyElement::DoDeclareParameters().
  void DoDeclareParameters(
      internal::MultibodyTreeSystem<T>* tree_system) override {
    // Declare parent class's parameters
    MultibodyElement<Joint, T, JointIndex>::DoDeclareParameters(tree_system);

    is_locked_parameter_index_ =
        this->DeclareAbstractParameter(tree_system, Value<bool>(false));
  }

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

  std::string name_;
  const Frame<T>& frame_on_parent_;
  const Frame<T>& frame_on_child_;

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

  // System parameter index for `this` joint's lock state stored in a context.
  systems::AbstractParameterIndex is_locked_parameter_index_;

  // The Joint<T> implementation:
  std::unique_ptr<JointImplementation> implementation_;
};

}  // namespace multibody
}  // namespace drake
