#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/random.h"
#include "drake/math/random_rotation.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/quaternion_floating_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to move freely relatively to one another.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frame M to translate and rotate freely with respect to F,
/// introducing six degrees of freedom. This Joint introduces four generalized
/// positions to describe the orientation `R_FM` of frame M in F with a
/// quaternion `q_FM`, and three generalized positions to describe the
/// translation of frame M's origin in F with a position vector `p_FM`. The
/// seven entries of the configuration vector q are ordered `(q_FM, p_FM)` with
/// the quaternion, ordered wxyz (scalar then vector), preceding the translation
/// vector. As generalized velocities, this Joint introduces the angular
/// velocity `w_FM` of frame M in F and the linear velocity `v_FM` of frame M's
/// origin in frame F, ordered `(w_FM, v_FM)`.
///
/// @tparam_default_scalar
template <typename T>
class QuaternionFloatingJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionFloatingJoint);

  /// The name for this Joint type.  It resolves to "quaternion_floating".
  static const char kTypeName[];

  /// Constructor for a %QuaternionFloatingJoint granting six degrees of
  /// freedom to an outboard frame M attached to the child body B with respect
  /// to an inboard frame F attached to the parent body P. The orientation of
  /// frame M in F is represented by a quaternion `q_FM` while the position of F
  /// in M is given by a position vector `p_FM` expressed in frame F. See this
  /// class's documentation for further details on the definition of these
  /// frames, get_quaternion() and get_translation() for an explanation of the
  /// configuration of this joint, and get_angular_velocity() and
  /// get_translational_velocity() for an explanation of the generalized
  /// velocities.
  ///
  /// This constructor signature creates a joint with no joint
  /// limits, i.e. the joint position, velocity and acceleration limits are the
  /// pair `(-∞, ∞)`. These can be set using the Joint methods
  /// set_position_limits(), set_velocity_limits() and
  /// set_acceleration_limits().
  ///
  /// The first three arguments to this constructor
  /// are those of the Joint class constructor. See the Joint class's
  /// documentation for details. The additional parameters are:
  /// @param[in] angular_damping
  ///  Viscous damping coefficient in N⋅m⋅s for the angular component of
  ///  this joint's velocity, used to model losses within the joint. See
  ///  documentation of default_angular_damping() for details on modelling of
  ///  the damping force.
  /// @param[in] translational_damping
  ///  Viscous damping coefficient in N⋅s/m for the translational component of
  ///  this joint's velocity, used to model losses within the joint. See
  ///  documentation of default_translational_damping() for details on modelling
  ///  of the damping force.
  /// @throws std::exception if angular_damping is negative.
  /// @throws std::exception if translational_damping is negative.
  QuaternionFloatingJoint(const std::string& name,
                          const Frame<T>& frame_on_parent,
                          const Frame<T>& frame_on_child,
                          double angular_damping = 0.0,
                          double translational_damping = 0.0)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 (Vector6<double>() << angular_damping, angular_damping,
                  angular_damping, translational_damping, translational_damping,
                  translational_damping)
                     .finished(),
                 Vector<double, 7>::Constant(
                     -std::numeric_limits<double>::infinity()),
                 Vector<double, 7>::Constant(
                     std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS(angular_damping >= 0);
    DRAKE_THROW_UNLESS(translational_damping >= 0);
    // Parent constructor sets all default positions to 0.
    // Adjust quaternion default to identity.
    this->set_default_quaternion(Quaternion<double>::Identity());
  }

  ~QuaternionFloatingJoint() final;

  /// Returns the name of this joint type: "quaternion_floating"
  const std::string& type_name() const final {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns `this` joint's default angular damping constant in N⋅m⋅s. The
  /// damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e. opposing
  /// motion, with ω the angular velocity of frame M in F (see
  /// get_angular_velocity()) and τ the torque on child body B (to which M is
  /// rigidly attached).
  double default_angular_damping() const {
    // N.B. All 3 angular damping coefficients are set to the same value for
    // this joint.
    return this->default_damping_vector()[0];
  }

  /// Returns `this` joint's default translational damping constant in N⋅s/m.
  /// The damping force (in N) is modeled as `f = -damping⋅v` i.e. opposing
  /// motion, with v the translational velocity of frame M in F (see
  /// get_translational_velocity()) and f the force on child body B at Mo.
  double default_translational_damping() const {
    // N.B. All 3 translational damping coefficients are set to the same value
    // for this joint.
    return this->default_damping_vector()[3];
  }

  /// @name Context-dependent value access
  /// @{

  /// Gets the quaternion `q_FM` that represents the orientation of outboard
  /// frame M in the inboard frame F. Refer to the documentation for this class
  /// for details.
  /// @param[in] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @retval q_FM The quaternion representing the orientation of frame M in F.
  Quaternion<T> get_quaternion(const systems::Context<T>& context) const {
    return get_mobilizer().get_quaternion(context);
  }

  /// Returns the position vector p_FoMo_F from Fo (inboard frame F's origin)
  /// to Mo (outboard frame M's origin), expressed in inboard frame F.
  /// @param[in] context contains the state of the multibody system.
  /// @note Class documentation describes inboard frame F and outboard frame M.
  /// @retval p_FM The position vector from Fo (frame F's origin) to Mo (frame
  /// M's origin), expressed in frame F.
  Vector3<T> get_translation(const systems::Context<T>& context) const {
    return get_mobilizer().get_translation(context);
  }

  /// Returns the pose `X_FM` of the outboard frame M as measured and expressed
  /// in the inboard frame F. Refer to the documentation for this class for
  /// details.
  /// @param[in] context A Context for the MultibodyPlant this joint belongs to.
  /// @retval X_FM The pose of frame M in frame F.
  math::RigidTransform<T> GetPose(const systems::Context<T>& context) const {
    return math::RigidTransform<T>(get_quaternion(context),
                                   get_translation(context));
  }

  /// Retrieves from `context` the angular velocity `w_FM` of the child frame
  /// M in the parent frame F, expressed in F.
  /// @param[in] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @retval w_FM
  ///   A vector in ℝ³ with the angular velocity of the child frame M in the
  ///   parent frame F, expressed in F. Refer to this class's documentation for
  ///   further details and definitions of these frames.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_angular_velocity(context);
  }

  /// Retrieves from `context` the translational velocity `v_FM` of the
  /// child frame M's origin as measured and expressed in the parent frame F.
  /// @param[in] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @retval v_FM
  ///   A vector in ℝ³ with the translational velocity of the origin of child
  ///   frame M in the parent frame F, expressed in F. Refer to this class's
  ///   documentation for further details and definitions of these frames.
  Vector3<T> get_translational_velocity(
      const systems::Context<T>& context) const {
    return get_mobilizer().get_translational_velocity(context);
  }

  /// @}

  /// @name Context-dependent value setters
  /// @{

  /// Sets `context` so that the orientation of frame M in F is given by the
  /// input quaternion `q_FM`.
  /// @param[in,out] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @param[in] q_FM
  ///   Quaternion relating frames F and M to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const QuaternionFloatingJoint<T>& SetQuaternion(
      systems::Context<T>* context, const Quaternion<T>& q_FM) const {
    get_mobilizer().SetQuaternion(context, q_FM);
    return *this;
  }

  /// Sets the quaternion in `context` so this joint's orientation is consistent
  /// with the given `R_FM` rotation matrix.
  /// @param[in,out] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @param[in] R_FM
  ///   The rotation matrix relating the orientation of frame F and frame M.
  /// @returns a constant reference to `this` joint.
  const QuaternionFloatingJoint<T>& SetOrientation(
      systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const {
    get_mobilizer().SetOrientation(context, R_FM);
    return *this;
  }

  /// For this joint, stores the position vector `p_FM` in `context`.
  /// @param[in,out] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @param[in] p_FM position vector p_FoMo_F from Fo (inboard frame F's
  /// origin) to Mo (outboard frame M's origin), expressed in frame F.
  /// @returns a constant reference to `this` joint.
  const QuaternionFloatingJoint<T>& SetTranslation(
      systems::Context<T>* context, const Vector3<T>& p_FM) const {
    get_mobilizer().SetTranslation(context, p_FM);
    return *this;
  }

  /// Sets `context` to store `X_FM` the pose of frame M measured and expressed
  ///   in frame F.
  /// @param[in,out] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @param[in] X_FM
  ///   The desired pose of frame M in frame F to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const QuaternionFloatingJoint<T>& SetPose(
      systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const {
    SetTranslation(context, X_FM.translation());
    return SetOrientation(context, X_FM.rotation());
  }

  /// Sets in `context` the state for `this` joint so that the angular velocity
  /// of the child frame M in the parent frame F is `w_FM`.
  /// @param[in,out] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @param[in] w_FM
  ///   A vector in ℝ³ with the angular velocity of the child frame M in the
  ///   parent frame F, expressed in F. Refer to this class's documentation for
  ///   further details and definitions of these frames.
  /// @returns a constant reference to `this` joint.
  const QuaternionFloatingJoint<T>& set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const {
    get_mobilizer().SetAngularVelocity(context, w_FM);
    return *this;
  }

  /// Sets in `context` the state for `this` joint so that the translational
  /// velocity of the child frame M's origin in the parent frame F is `v_FM`.
  /// @param[in,out] context
  ///   A Context for the MultibodyPlant this joint belongs to.
  /// @param[in] v_FM
  ///   A vector in ℝ³ with the translational velocity of the child frame M's
  ///   origin in the parent frame F, expressed in F. Refer to this class's
  ///   documentation for further details and definitions of these frames.
  /// @returns a constant reference to `this` joint.
  const QuaternionFloatingJoint<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const {
    get_mobilizer().SetTranslationalVelocity(context, v_FM);
    return *this;
  }

  /// @}

  /// @name Random distribution setters
  /// @{

  /// For this joint, sets the random distribution that the translation of this
  /// joint will be randomly sampled from. If a quaternion distribution has
  /// already been set with stochastic variables, it will remain so. Otherwise
  /// the quaternion will be set to this joint's zero orientation.
  /// See get_translation() for details on the translation representation.
  void set_random_translation_distribution(
      const Vector3<symbolic::Expression>& p_FM) {
    get_mutable_mobilizer().set_random_translation_distribution(p_FM);
  }

  /// (Advanced) Sets the random distribution that the orientation of this joint
  /// will be randomly sampled from. If a translation (position) distribution
  /// has already been set with stochastic variables, it will remain so.
  /// Otherwise translation will be set to this joint's zero configuration.
  /// See get_quaternion() for details on the orientation representation.
  /// @note Use caution when setting a quaternion distribution. A naive uniform
  /// sampling of each component will not lead to a uniform sampling of the unit
  /// sphere. See `set_random_quaternion_distribution_to_uniform()` for the most
  /// common case of uniformly sampling rotations.
  void set_random_quaternion_distribution(
      const Eigen::Quaternion<symbolic::Expression>& q_FM) {
    get_mutable_mobilizer().set_random_quaternion_distribution(q_FM);
  }

  /// Sets the random distribution such that the orientation of this joint will
  /// be randomly sampled using uniformly sampled rotations.
  void set_random_quaternion_distribution_to_uniform() {
    RandomGenerator generator;
    auto q_FM =
        math::UniformlyRandomQuaternion<symbolic::Expression>(&generator);
    get_mutable_mobilizer().set_random_quaternion_distribution(q_FM);
  }

  /// @}

  /// @name Default value getters
  /// @{

  /// Gets the default quaternion `q_FM` for `this` joint.
  /// @returns The default quaternion `q_FM` of `this`.
  Quaternion<double> get_default_quaternion() const {
    const Vector4<double>& q_FM = this->default_positions().template head<4>();
    return Quaternion<double>(q_FM[0], q_FM[1], q_FM[2], q_FM[3]);
  }

  /// Returns this joint's default translation as the position vector p_FoMo_F
  /// from Fo (inboard frame F's origin) to Mo (outboard frame M's origin),
  /// expressed in inboard frame F.
  /// @retval This joint's default translation as the position vector p_FM.
  Vector3<double> get_default_translation() const {
    return this->default_positions().template tail<3>();
  }
  /// @}

  /// @name Default value setters
  /// @{

  /// Sets the default quaternion `q_FM` of this joint.
  /// @param[in] q_FM
  ///   The desired default quaternion of the joint.
  void set_default_quaternion(const Quaternion<double>& q_FM) {
    VectorX<double> default_positions = this->default_positions();
    // @note we store the quaternion components consistently with
    // `QuaternionFloatingMobilizer<T>::get_quaternion()`
    default_positions[0] = q_FM.w();
    default_positions[1] = q_FM.x();
    default_positions[2] = q_FM.y();
    default_positions[3] = q_FM.z();
    this->set_default_positions(default_positions);
  }

  /// Sets this joint's default position vector `p_FM`.
  /// @param[in] p_FM position vector p_FoMo_F from Fo (inboard frame F's
  /// origin) to Mo (outboard frame M's origin), expressed in frame F.
  void set_default_translation(const Vector3<double>& p_FM) {
    VectorX<double> default_positions = this->default_positions();
    default_positions.template tail<3>() = p_FM;
    this->set_default_positions(default_positions);
  }
  /// @}

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Adding forces per-dof makes no physical sense. Therefore, this method
  /// throws an exception if invoked.
  void DoAddInOneForce(const systems::Context<T>&, int, const T&,
                       MultibodyForces<T>*) const final;

  /// Joint<T> override called through public NVI, Joint::AddInDamping().
  /// Therefore arguments were already checked to be valid.
  /// This method adds into the translational component of `forces` for `this`
  /// joint a dissipative force according to the viscous law `f = -d⋅v`, with d
  /// the damping coefficient (see default_translational_damping()). This method
  /// also adds into the angular component of `forces` for `this` joint a
  /// dissipative torque according to the viscous law `τ = -d⋅ω`, with d the
  /// damping coefficient (see default_angular_damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final;

 private:
  int do_get_velocity_start() const final {
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 6; }

  int do_get_position_start() const final {
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const final { return 7; }

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
    VectorX<double> q(7);
    q << q_FM.w(), q_FM.vec(), p_FM;
    this->set_default_positions(q);
  }

  std::pair<Eigen::Quaternion<double>, Vector3<double>> DoGetDefaultPosePair()
      const final {
    const VectorX<double>& q = this->default_positions();
    return std::make_pair(Eigen::Quaternion<double>(q[0], q[1], q[2], q[3]),
                          q.tail<3>());
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

  // Make QuaternionFloatingJoint templated on every other scalar type a friend
  // of QuaternionFloatingJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can
  // access private members of QuaternionFloatingJoint<T>.
  template <typename>
  friend class QuaternionFloatingJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::QuaternionFloatingMobilizer<T>& get_mobilizer() const {
    return this->template get_mobilizer_downcast<
        internal::QuaternionFloatingMobilizer>();
  }

  internal::QuaternionFloatingMobilizer<T>& get_mutable_mobilizer() {
    return this->template get_mutable_mobilizer_downcast<
        internal::QuaternionFloatingMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;
};

template <typename T>
const char QuaternionFloatingJoint<T>::kTypeName[] = "quaternion_floating";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::QuaternionFloatingJoint);
