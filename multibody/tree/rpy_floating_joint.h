#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/rpy_floating_mobilizer.h"

namespace drake {
namespace multibody {

/** This Joint allows a rigid body to move freely with respect to its parent
rigid body. This is most commonly used to allow a body to move freely with
respect to the World, but can be used with any parent. More precisely, given a
frame F attached to the parent body P and a frame M attached to the child body
B (see the Joint class's documentation), this joint allows frame M to translate
and rotate freely with respect to F, introducing six degrees of freedom.
However, unlike the QuaternionFloatingJoint, the orientation of M relative to
F is parameterized with roll-pitch-yaw angles (see warning below). The
generalized coordinates q for this joint are the three orientation angles
followed by three generalized positions to describe the translation of frame
M's origin Mo in F with a position vector `p_FM`. As generalized velocities, we
use the angular velocity `w_FM` of frame M in F (_not_ the orientation angle
time derivatives q̇) and the linear velocity `v_FM` of frame M's origin Mo in
frame F.

@warning Any three-parameter representation of orientation necessarily has a
singularity somewhere. In this case, the singularity occurs when the pitch angle
(second generalized coordinate q) is at π/2 + kπ (for any integer k), and
numerical issues may occur when near one of those configurations. If you can't
be sure your simulation will avoid the singularities, consider using the
singularity-free QuaternionFloatingJoint instead.

@tparam_default_scalar */
template <typename T>
class RpyFloatingJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RpyFloatingJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  /// The name for this Joint type.  It resolves to "rpy_floating".
  static const char kTypeName[];

  /** Constructor to create an rpy floating joint between two bodies so that
  frame F attached to the parent body P and frame M attached to the child body
  B move freely relative to one another. See this class's documentation for
  further details on the definition of these frames and the generalized
  positions q and generalized velocities v for this joint. This constructor
  signature creates a joint with no joint limits, i.e. the joint position,
  velocity and acceleration limits are the pair `(-∞, ∞)`. These can be set
  using the Joint methods set_position_limits(), set_velocity_limits() and
  set_acceleration_limits().

  The first three arguments to this constructor are those of the Joint class
  constructor. See the Joint class's documentation for details. The additional
  parameters are:
  @param[in] angular_damping
    Viscous damping coefficient in N⋅m⋅s for the angular component of this
    joint's velocity, used to model losses within the joint. See documentation
    of default_angular_damping() for details on modeling of the damping force.
  @param[in] translational_damping
    Viscous damping coefficient in N⋅s/m for the translational component of
    this joint's velocity, used to model losses within the joint. See
    documentation of default_translational_damping() for details on modeling of
  the damping force.
  @throws std::exception if angular_damping is negative.
  @throws std::exception if translational_damping is negative. */
  RpyFloatingJoint(const std::string& name, const Frame<T>& frame_on_parent,
                   const Frame<T>& frame_on_child, double angular_damping = 0.0,
                   double translational_damping = 0.0)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 (Vector6d() << angular_damping, angular_damping,
                  angular_damping, translational_damping, translational_damping,
                  translational_damping)
                     .finished(),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS(angular_damping >= 0);
    DRAKE_THROW_UNLESS(translational_damping >= 0);

    // Parent constructor sets all default positions to zero which is correct
    // for this joint.
  }

  ~RpyFloatingJoint() final;

  /** Returns the name of this joint type: "rpy_floating" */
  const std::string& type_name() const final;

  /** Returns this joint's default angular damping constant in N⋅m⋅s. The
  damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e. opposing motion,
  with ω the angular velocity of frame M in F (see get_angular_velocity()) and τ
  the torque on child body B (to which M is rigidly attached). */
  double default_angular_damping() const {
    // N.B. All 3 angular damping coefficients are set to the same value for
    // this joint.
    return this->default_damping_vector()[0];
  }

  /** Returns this joint's default translational damping constant in N⋅s/m. The
  damping force (in N) is modeled as `f = -damping⋅v` i.e. opposing motion,
  with v the translational velocity of frame M in F (see
  get_translational_velocity()) and f the force on child body B at Mo. */
  double default_translational_damping() const {
    // N.B. All 3 translational damping coefficients are set to the same value
    // for this joint.
    return this->default_damping_vector()[3];
  }

  /** @name Context-dependent value access
  Functions in this section are given a Context and either get from it, or
  set in it, quantities relevant to this %RpyFloatingJoint. These functions can
  only be called after MultibodyPlant::Finalize(). */
  /**@{*/

  /** Gets the roll-pitch-yaw rotation angles of this joint from `context`.

  The orientation `R_FM` of the child frame M in parent frame F is
  parameterized with roll-pitch-yaw angles (also known as space-fixed
  x-y-z Euler angles and extrinsic angles). That is, the angles θr, θp, θy,
  correspond to a sequence of rotations about the Fx, Fy, and Fz axes of
  parent frame F, respectively. Mathematically, rotation `R_FM` is given in
  terms of angles θr, θp, θy by: <pre>
    R_FM(q) = Rz(θy) * Ry(θp) * Rx(θr)
  </pre>
  where `Rx(θ)`, `Ry(θ)` and `Rz(θ)` correspond to the elemental rotations in
  amount of θ about the Fx, Fy and Fz axes respectively. Zero θr, θp, θy angles
  corresponds to frames F and M having the same orientation (their origins may
  still be separated). Angles θr, θp, θy are defined to be positive according
  to the right-hand-rule with the thumb aligned in the direction of their
  respective axes.

  @note Space `x-y-z` angles (extrinsic) are equivalent to Body `z-y-x`
  angles (intrinsic).

  @param[in] context
    A Context for the MultibodyPlant this joint belongs to.
  @retval angles The angle coordinates of this joint stored in the `context`
    ordered as θr, θp, θy. */
  Vector3<T> get_angles(const Context<T>& context) const {
    return get_mobilizer().get_angles(context);
  }

  /** Sets the `context` so that the generalized coordinates corresponding to
  the roll-pitch-yaw rotation angles of this joint equals `angles`.
  @param[in,out] context
    A Context for the MultibodyPlant this joint belongs to.
  @param[in] angles
    Angles in radians to be stored in `context` ordered as θr, θp, θy.
  @warning See class documentation for discussion of singular configurations.
  @returns a constant reference to this joint.
  @see get_angles() for details */
  const RpyFloatingJoint<T>& set_angles(Context<T>* context,
                                        const Vector3<T>& angles) const {
    get_mobilizer().SetAngles(context, angles);
    return *this;
  }

  /** Sets the roll-pitch-yaw angles in `context` so this Joint's orientation
  is consistent with the given `R_FM` rotation matrix.
  @param[in,out] context
    A Context for the MultibodyPlant this joint belongs to.
  @param[in] R_FM
    The rotation matrix giving the orientation of frame M in frame F.
  @warning See class documentation for discussion of singular configurations.
  @returns a constant reference to this joint. */
  const RpyFloatingJoint<T>& SetOrientation(
      systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const {
    return set_angles(context, math::RollPitchYaw(R_FM).vector());
  }

  /** Returns the translation (position vector) `p_FM` of the child frame M's
  origin Mo as measured and expressed in the parent frame F. Refer to the
  class documentation for details.
  @param[in] context
    A Context for the MultibodyPlant this joint belongs to.
  @retval p_FM The position vector of frame M's origin in frame F. */
  Vector3<T> get_translation(const systems::Context<T>& context) const {
    return get_mobilizer().get_translation(context);
  }

  /** Sets `context` to store the translation (position vector) `p_FM` of frame
  M's origin Mo measured and expressed in frame F.
  @param[in,out] context
    A Context for the MultibodyPlant this joint belongs to.
  @param[in] p_FM
    The desired position of frame M's origin in frame F, to be stored in
    `context`.
  @returns a constant reference to this joint. */
  const RpyFloatingJoint<T>& SetTranslation(systems::Context<T>* context,
                                            const Vector3<T>& p_FM) const {
    get_mobilizer().SetTranslation(context, p_FM);
    return *this;
  }

  /** Returns the pose `X_FM` of the outboard frame M as measured and expressed
  in the inboard frame F. Refer to the documentation for this class for
  details.
  @param[in] context
    A Context for the MultibodyPlant this joint belongs to.
  @retval X_FM The pose of frame M in frame F. */
  math::RigidTransform<T> GetPose(const systems::Context<T>& context) const {
    return math::RigidTransform<T>(math::RollPitchYaw<T>(get_angles(context)),
                                   get_translation(context));
  }

  /** Sets `context` to store `X_FM` the pose of frame M measured and expressed
  in frame F.
  @param[in,out] context
    A Context for the MultibodyPlant this joint belongs to.
  @param[in] X_FM
    The desired pose of frame M in frame F to be stored in `context`.
  @warning See class documentation for discussion of singular configurations.
  @returns a constant reference to `this` joint. */
  const RpyFloatingJoint<T>& SetPose(
      systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const {
    const math::RotationMatrix<T>& R_FM = X_FM.rotation();
    get_mobilizer().SetAngles(context, math::RollPitchYaw<T>(R_FM).vector());
    get_mobilizer().SetTranslation(context, X_FM.translation());
    return *this;
  }

  /** Retrieves from `context` the angular velocity `w_FM` of the child frame
  M in the parent frame F, expressed in F.
  @param[in] context
    A Context for the MultibodyPlant this joint belongs to.
  @retval w_FM
    A vector in ℝ³ with the angular velocity of the child frame M in the
    parent frame F, expressed in F. Refer to this class's documentation for
    further details and definitions of these frames. */
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_angular_velocity(context);
  }

  /** Sets in `context` the state for this joint so that the angular velocity
  of the child frame M in the parent frame F is `w_FM`.
  @param[in,out] context
    A Context for the MultibodyPlant this joint belongs to.
  @param[in] w_FM
    A vector in ℝ³ with the angular velocity of the child frame M in the
    parent frame F, expressed in F. Refer to this class's documentation for
    further details and definitions of these frames.
  @returns a constant reference to this joint. */
  const RpyFloatingJoint<T>& set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const {
    get_mobilizer().SetAngularVelocity(context, w_FM);
    return *this;
  }

  /** Retrieves from `context` the translational velocity `v_FM` of
  the child frame M's origin as measured and expressed in the parent frame F.
  @param[in] context
    A Context for the MultibodyPlant this joint belongs to.
  @retval v_FM
    A vector in ℝ³ with the translational velocity of the origin of child
    frame M in the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames. */
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const {
    return get_mobilizer().get_translational_velocity(context);
  }

  /** Sets in `context` the state for this joint so that the translational
  velocity of the child frame M's origin in the parent frame F is `v_FM`.
  @param[in,out] context
    A Context for the MultibodyPlant this joint belongs to.
  @param[in] v_FM
    A vector in ℝ³ with the translational velocity of the child frame M's
    origin in the parent frame F, expressed in F. Refer to this class's
    documentation for further details and definitions of these frames.
  @returns a constant reference to this joint. */
  const RpyFloatingJoint<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const {
    get_mobilizer().SetTranslationalVelocity(context, v_FM);
    return *this;
  }
  /**@}*/

  /** @name Random distribution setters
  These functions can only be called after MultibodyPlant::Finalize(). */
  /**@{*/

  /** Sets the random distribution from which the roll-pitch-yaw orientation
  angles of this joint will be randomly sampled. See the %RpyFloatingJoint class
  documentation for details on the orientation representation. If a translation
  distribution has already been set with stochastic variables, it will remain
  so. Otherwise translation will be set to this joint's zero configuration.
  @warning Watch for random pitch angles near the singular configuration for
    this joint (see class documentation). */
  void set_random_angles_distribution(
      const Vector3<symbolic::Expression>& angles) {
    get_mutable_mobilizer().set_random_angles_distribution(angles);
  }

  /** Sets the random distribution that the translation vector of this joint
  will be randomly sampled from. See the %RpyFloatingJoint class documentation
  for details on the translation representation. If an angles distribution has
  has already been set with stochastic variables, it will remain so. Otherwise
  angles will be set to this joint's zero configuration. */
  void set_random_translation_distribution(
      const Vector3<symbolic::Expression>& p_FM) {
    get_mutable_mobilizer().set_random_translation_distribution(p_FM);
  }
  /**@}*/

  /** @name Default pose access
  Functions in this section set or get the default values for the pose states q
  (rpy angles and translation vector) of this %RpyFloatingJoint. These are
  stored directly in the joint rather than in a Context and are used to
  initialize Contexts. Note that the default velocities v are always zero. */
  /**@{*/

  /** Gets the default angles for this joint.
  @retval angles The default roll-pitch-yaw angles as a Vector3. */
  Vector3<double> get_default_angles() const {
    return this->default_positions().template head<3>();
  }

  /** Sets the default roll-pitch-yaw angles of this joint.
  @param[in] angles the desired default angles of the joint
  @warning See class documentation for discussion of singular configurations. */
  void set_default_angles(const Vector3<double>& angles) {
    VectorX<double> default_positions = this->default_positions();
    default_positions.template head<3>() = angles;
    this->set_default_positions(default_positions);
  }

  /** Gets the default translation (position vector) `p_FM` for this joint.
  @retval p_FM The default translation of this joint. */
  Vector3<double> get_default_translation() const {
    return this->default_positions().template tail<3>();
  }

  /** Sets the default translation (position vector) `p_FM` of this joint.
  @param[in] p_FM The desired default translation of the joint. */
  void set_default_translation(const Vector3<double>& p_FM) {
    VectorX<double> default_positions = this->default_positions();
    default_positions.template tail<3>() = p_FM;
    this->set_default_positions(default_positions);
  }
  /**@}*/

 protected:
  /** Joint<T> override called through public NVI, Joint::AddInForce().
  Adding forces per-dof for this joint is not supported. Therefore, this method
  throws an exception if invoked. */
  void DoAddInOneForce(const systems::Context<T>&, int, const T&,
                       MultibodyForces<T>*) const final {
    throw std::logic_error(
        "RpyFloating joints do not allow applying forces to individual "
        "degrees of freedom.");
  }

  /** Joint<T> override called through public NVI, Joint::AddInDamping().
  Therefore arguments were already checked to be valid. This method adds into
  the translational component of `forces` for `this` joint a dissipative force
  according to the viscous law `f = -d⋅v`, with d the damping coefficient (see
  default_translational_damping()). This method also adds into the angular
  component of `forces` for `this` joint a dissipative torque according to the
  viscous law `τ = -d⋅ω`, with d the damping coefficient (see
  default_angular_damping()). */
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final {
    Eigen::Ref<VectorX<T>> t_BMo_F =
        get_mobilizer().get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    const Vector3<T>& w_FM = get_angular_velocity(context);
    const Vector3<T>& v_FM = get_translational_velocity(context);
    const T& angular_damping = this->GetDampingVector(context)[0];
    const T& translational_damping = this->GetDampingVector(context)[3];
    t_BMo_F.template head<3>() = -angular_damping * w_FM;
    t_BMo_F.template tail<3>() = -translational_damping * v_FM;
  }

 private:
  int do_get_velocity_start() const final {
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 6; }

  int do_get_position_start() const final {
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const final { return 6; }

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

  void DoSetDefaultPosePair(const Quaternion<double>& q_FM,
                            const Vector3<double>& p_FM) final {
    const math::RollPitchYaw<double> rpy_FM(q_FM);
    VectorX<double> q(6);
    q << rpy_FM.vector(), p_FM;
    this->set_default_positions(q);
  }

  std::pair<Eigen::Quaternion<double>, Vector3<double>> DoGetDefaultPosePair()
      const final {
    const VectorX<double>& q = this->default_positions();
    const math::RollPitchYaw<double> rpy_FM(q.head<3>());
    return std::make_pair(rpy_FM.ToQuaternion(), q.tail<3>());
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

  // Make RpyFloatingJoint templated on every other scalar type a friend of
  // RpyFloatingJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of RpyFloatingJoint<T>.
  template <typename>
  friend class RpyFloatingJoint;

  // Returns the mobilizer implementing this joint.
  const internal::RpyFloatingMobilizer<T>& get_mobilizer() const {
    return this
        ->template get_mobilizer_downcast<internal::RpyFloatingMobilizer>();
  }

  internal::RpyFloatingMobilizer<T>& get_mutable_mobilizer() {
    return this->template get_mutable_mobilizer_downcast<
        internal::RpyFloatingMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;
};

template <typename T>
const char RpyFloatingJoint<T>::kTypeName[] = "rpy_floating";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RpyFloatingJoint);
