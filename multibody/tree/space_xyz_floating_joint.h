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
#include "drake/math/roll_pitch_yaw.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/space_xyz_floating_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to move freely relatively to one another.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frame M to translate and rotate freely with respect to F,
/// introducing six degrees of freedom. This Joint introduces four generalized
/// positions to describe the orientation `R_FM` of frame M in F with a
/// quaternion `q_FM`, and three generalized positions to describe the position
/// of frame M's origin in F with a position vector `p_FM`. As generalized
/// velocities, this Joint introduces the angular velocity `w_FM` of frame M in
/// F and the linear velocity `v_FM` of frame M's origin in frame F.
///
/// @tparam_default_scalar
template <typename T>
class SpaceXYZFloatingJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SpaceXYZFloatingJoint)

  /// The name for this Joint type.  It resolves to "quaternion_floating".
  static const char kTypeName[];

  /// Constructor for a %SpaceXYZFloatingJoint granting six degrees of
  /// freedom to an outboard frame M attached to the child body B with respect
  /// to an inboard frame F attached to the parent body P. The orientation of
  /// frame M in F is represented by a quaternion `q_FM` while the position of F
  /// in M is given by a position vector `p_FM` expressed in frame F. See this
  /// class's documentation for further details on the definition of these
  /// frames, get_quaternion() and get_position() for an explanation of the
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
  ///  documentation of angular_damping() for details on modelling of the
  ///  damping force.
  /// @param[in] translational_damping
  ///  Viscous damping coefficient in N⋅s/m for the translational component of
  ///  this joint's velocity, used to model losses within the joint. See
  ///  documentation of translational_damping() for details on modelling of the
  ///  damping force.
  /// @throws std::exception if angular_damping is negative.
  /// @throws std::exception if translational_damping is negative.
  SpaceXYZFloatingJoint(const std::string& name,
                          const Frame<T>& frame_on_parent,
                          const Frame<T>& frame_on_child,
                          double angular_damping = 0.0,
                          double translational_damping = 0.0)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 (Vector6<double>() << angular_damping, angular_damping,
                  angular_damping, translational_damping, translational_damping,
                  translational_damping)
                     .finished(),
                 Vector<double, 6>::Constant(
                     -std::numeric_limits<double>::infinity()),
                 Vector<double, 6>::Constant(
                     std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(-std::numeric_limits<double>::infinity()),
                 Vector6d::Constant(std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS(angular_damping >= 0);
    DRAKE_THROW_UNLESS(translational_damping >= 0);
  }

  /// Returns the name of this joint type: "space_xyz_floating"
  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns `this` joint's angular damping constant in N⋅m⋅s. The damping
  /// torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e. opposing motion, with
  /// ω the angular velocity of frame M in F (see get_angular_velocity()) and τ
  /// the torque on child body B (to which M is rigidly attached).
  double angular_damping() const {
    // N.B. All 3 angular damping coefficients are set to the same value for
    // this joint.
    return this->damping_vector()[0];
  }

  /// Returns `this` joint's translational damping constant in N⋅s/m. The
  /// damping force (in N) is modeled as `f = -damping⋅v` i.e. opposing motion,
  /// with v the translational velocity of frame M in F (see
  /// get_translational_velocity()) and f the force on child body B at Mo.
  double translational_damping() const {
    // N.B. All 3 translational damping coefficients are set to the same value
    // for this joint.
    return this->damping_vector()[3];
  }

  /// @name Context-dependent value access
  /// @{

  /// Gets the quaternion `q_FM` that represents the orientation of outboard
  /// frame M in the inboard frame F. Refer to the documentation for this class
  /// for details.
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @returns The quaternion representing the orientation of frame M in F.
  Vector3<T> get_angles(const systems::Context<T>& context) const {
    return get_mobilizer().get_angles(context);
  }

  /// Returns the position `p_FM` of the outboard frame M's origin as measured
  /// and expressed in the inboard frame F. Refer to the documentation for this
  /// class for details.
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @returns The position vector of frame M's origin in frame F.
  Vector3<T> get_position(const systems::Context<T>& context) const {
    // N.B. Though the mobilizer accessor is get_translation, this method is
    // named get_position() for consistency with QuaternionFloatingJoint.
    return get_mobilizer().get_translation(context);
  }

  /// Returns the pose `X_FM` of the outboard frame M as measured and expressed
  /// in the inboard frame F. Refer to the documentation for this class for
  /// details.
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @returns The pose of frame M in frame F.
  math::RigidTransform<T> get_pose(const systems::Context<T>& context) const {
    return math::RigidTransform<T>(math::RollPitchYaw(get_angles(context)),
                                   get_position(context));
  }

  /// Retrieves from `context` the angular velocity `w_FM` of the child frame
  /// M in the parent frame F, expressed in F.
  /// @param[in] context
  ///   The context of the model this joint belongs to.
  /// @retval w_FM
  ///   A vector in ℝ³ with the angular velocity of the child frame M in the
  ///   parent frame F, expressed in F. Refer to this class's documentation for
  ///   further details and definitions of these frames.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_angular_velocity(context);
  }

  /// Retrieves from `context` the translational velocity `v_FM` of
  /// the child frame M's origin as measured and expressed in the parent frame
  /// F.
  /// @param[in] context
  ///   The context of the model this joint belongs to.
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
  /// @param[out] context
  ///   The context of the model this joint belongs to.
  /// @param[in] q_FM
  ///   The desired orientation of M in F to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const SpaceXYZFloatingJoint<T>& set_angles(
      systems::Context<T>* context, const Vector3<T>& angles) const {
    get_mobilizer().set_angles(context, angles);
    return *this;
  }

  // Sets `context` so this Joint's orientation is consistent with the given
  // `R_FM` rotation matrix.
  // @param[in] context
  ///   The context of the model this joint belongs to.
  // @param[in] R_FM
  //   The rotation matrix relating the orientation of frame F and frame M.
  // @returns a constant reference to `this` joint.
  // @note: To create a RotationMatrix R_FM (which is inherently orthonormal)
  // from a non-orthonormal Matrix3<T> m (e.g., m is approximate data), use
  // R_FM = math::RotationMatrix<T>::ProjectToRotationMatrix( m ).
  // Alternatively, set this joint's orientation with the two statements:
  // const Eigen::Quaternion<T> q_FM = RotationMatrix<T>::ToQuaternion( m );
  // set_quaternion(context, q_FM);
  const SpaceXYZFloatingJoint<T>& SetFromRotationMatrix(
      systems::Context<T>* context, const math::RotationMatrix<T>& R_FM) const {
    const math::RigidTransform<T> X_FM(R_FM, get_position(*context));
    get_mobilizer().SetFromRigidTransform(context, X_FM);
    return *this;
  }

  /// Sets `context` to store the position `p_FM` of frame M's origin `Mo`
  /// measured and expressed in frame F.
  /// @param[out] context
  ///   The context of the model this joint belongs to.
  /// @param[in] p_FM
  ///   The desired position of frame M in F to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const SpaceXYZFloatingJoint<T>& set_position(systems::Context<T>* context,
                                                 const Vector3<T>& p_FM) const {
    // N.B. Though the mobilizer accessor is set_translation, this method is
    // named set_position() for consistency with QuaternionFloatingJoint.                                                    
    get_mobilizer().set_translation(context, p_FM);
    return *this;
  }

  /// Sets `context` to store `X_FM` the pose of frame M measured and expressed
  /// in frame F.
  /// @param[out] context
  ///   The context of the model this joint belongs to.
  /// @param[in] X_FM
  ///   The desired pose of frame M in F to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const SpaceXYZFloatingJoint<T>& set_pose(
      systems::Context<T>* context, const math::RigidTransform<T>& X_FM) const {
    set_position(context, X_FM.translation());
    set_angles(context, X_FM.rotation().ToRollPitchYaw().vector());
    return *this;
  }

  /// Sets in `context` the state for `this` joint so that the angular velocity
  /// of the child frame M in the parent frame F is `w_FM`.
  /// @param[out] context
  ///   The context of the model this joint belongs to.
  /// @param[in] w_FM
  ///   A vector in ℝ³ with the angular velocity of the child frame M in the
  ///   parent frame F, expressed in F. Refer to this class's documentation for
  ///   further details and definitions of these frames.
  /// @returns a constant reference to `this` joint.
  const SpaceXYZFloatingJoint<T>& set_angular_velocity(
      systems::Context<T>* context, const Vector3<T>& w_FM) const {
    get_mobilizer().set_angular_velocity(context, w_FM);
    return *this;
  }

  /// Sets in `context` the state for `this` joint so that the translational
  /// velocity of the child frame M's origin in the parent frame F is `v_FM`.
  /// @param[out] context
  ///   The context of the model this joint belongs to.
  /// @param[in] w_FM
  ///   A vector in ℝ³ with the translational velocity of the child frame M's
  ///   origin in the parent frame F, expressed in F. Refer to this class's
  ///   documentation for further details and definitions of these frames.
  /// @returns a constant reference to `this` joint.
  const SpaceXYZFloatingJoint<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector3<T>& v_FM) const {
    get_mobilizer().set_translational_velocity(context, v_FM);
    return *this;
  }

  /// @}

  /// @name Random distribution setters


  // TODO: implement these.

#if 0  
  /// Sets the random distribution that positions of this joint will be randomly
  /// sampled from. See get_position() for details on the position
  /// representation.
  void set_random_position_distribution(
      const Vector3<symbolic::Expression>& p_FM) {
    // TODO: fix this here and in QuanternionFloatingMobilizer, since this
    // mobilizer method mutates ALL positions (6 or 7) and the argument is size
    // 3.
    get_mutable_mobilizer()->set_random_position_distribution(p_FM);
  }

  /// (Advanced) Sets the random distribution that the orientation of this joint
  /// will be randomly sampled from. See get_quaternion() for details on the
  /// orientation representation.
  /// @note Use caution when setting a quaternion distribution. A naive uniform
  /// sampling of each component will not lead to a uniform sampling of the unit
  /// sphere. See `set_random_quaternion_distribution_to_uniform()` for the most
  /// common case of uniformly sampling rotations.
  void set_random_quaternion_distribution(
      const Vector3<symbolic::Expression>& rpy) {
    get_mutable_mobilizer()->set_random_quaternion_distribution(q_FM);
  }

  /// Sets the random distribution such that the orientation of this joint will
  /// be randomly sampled using uniformly sampled rotations.
  void set_random_quaternion_distribution_to_uniform() {
    RandomGenerator generator;
    auto q_FM =
        math::UniformlyRandomRPY<symbolic::Expression>(&generator);
    get_mutable_mobilizer()->set_random_quaternion_distribution(q_FM);
  }
#endif  

  /// @}

  /// @name Default value getters
  /// @{

  /// Gets the default quaternion `q_FM` for `this` joint.
  /// @returns The default quaternion `q_FM` of `this`.
  Vector3<double> get_default_angles() const {
    const Vector3<double>& rpy_FM =
        this->default_positions().template head<3>();
    return rpy_FM;
  }

  /// Gets the default position `p_FM` for `this` joint.
  /// @returns The default position `p_FM` of `this` joint.
  Vector3<double> get_default_position() const {
    return this->default_positions().template tail<3>();
  }

  /// Gets the default pose `X_FM` for `this` joint.
  /// @returns The default pose `X_FM` of `this` joint.
  math::RigidTransform<double> get_default_pose() const {
    return math::RigidTransform(math::RollPitchYaw(get_default_angles()),
                                get_default_position());
  }

  /// @}

  /// @name Default value setters
  /// @{

  /// Sets the default quaternion `q_FM` of this joint.
  /// @param[in] q_FM
  ///   The desired default quaternion of the joint.
  void set_default_angles(const Vector3<double>& rpy) {
    VectorX<double> default_positions = this->default_positions();
    // @note we store the quaternion components consistently with
    // `SpaceXYZFloatingMobilizer<T>::get_quaternion()`
    default_positions[0] = rpy[0];
    default_positions[1] = rpy[1];
    default_positions[2] = rpy[2];
    this->set_default_positions(default_positions);
  }

  /// Sets the default position `p_FM` of this joint.
  /// @param[in] p_FM
  ///   The desired default position of the joint.
  void set_default_position(const Vector3<double>& p_FM) {
    VectorX<double> default_positions = this->default_positions();
    default_positions.template tail<3>() = p_FM;
    this->set_default_positions(default_positions);
  }

  /// Sets the default pose `X_FM` of this joint.
  /// @param[in] X_FM
  ///   The desired default pose of the joint.
  void SetDefaultPose(const math::RigidTransform<double>& X_FM) {
    Vector<double, 6> default_positions;
    set_default_position(X_FM.translation());
    set_default_angles(X_FM.rotation().ToRollPitchYaw().vector());
  }

  /// @}

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Adding forces per-dof makes no physical sense. Therefore, this method
  /// throws an exception if invoked.
  void DoAddInOneForce(const systems::Context<T>&, int, const T&,
                       MultibodyForces<T>*) const override;

  /// Joint<T> override called through public NVI, Joint::AddInDamping().
  /// Therefore arguments were already checked to be valid.
  /// This method adds into the translational component of `forces` for `this`
  /// joint a dissipative force according to the viscous law `f = -d⋅v`, with d
  /// the damping coefficient (see translational_damping()). This method also
  /// adds into the angular component of `forces` for `this` joint a dissipative
  /// torque according to the viscous law `τ = -d⋅ω`, with d the damping
  /// coefficient (see angular_damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const override;

 private:
  int do_get_velocity_start() const override {
    return get_mobilizer().velocity_start_in_v();
  }

  int do_get_num_velocities() const override { return 6; }

  int do_get_position_start() const override {
    return get_mobilizer().position_start_in_q();
  }

  int do_get_num_positions() const override { return 6; }

  std::string do_get_position_suffix(int index) const override {
    return get_mobilizer().position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const override {
    return get_mobilizer().velocity_suffix(index);
  }

  void do_set_default_positions(
      const VectorX<double>& default_positions) override {
    if (this->has_implementation()) {
      get_mutable_mobilizer()->set_default_position(default_positions);
    }
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint()
      const override;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  // Make SpaceXYZFloatingJoint templated on every other scalar type a friend
  // of SpaceXYZFloatingJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can
  // access private members of SpaceXYZFloatingJoint<T>.
  template <typename>
  friend class SpaceXYZFloatingJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::SpaceXYZFloatingMobilizer<T>& get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::SpaceXYZFloatingMobilizer<T>* mobilizer =
        dynamic_cast<const internal::SpaceXYZFloatingMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return *mobilizer;
  }

  internal::SpaceXYZFloatingMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::SpaceXYZFloatingMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;
};

template <typename T>
const char SpaceXYZFloatingJoint<T>::kTypeName[] = "space_xyz_floating";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::SpaceXYZFloatingJoint)
