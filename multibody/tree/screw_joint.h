#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/screw_mobilizer.h"

namespace drake {
namespace multibody {

/// This joint models a screw joint allowing two bodies to rotate
/// about one axis while translating along that same axis with one degree of
///  freedom.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation), this
/// joint allows frame M to translate (while rotating) along an axis â.
/// Axis â is constant and has the same measures in both frames F and M, that
/// is, `â_F = â_M`. The rotation about the `â_F` axis and its rate
/// specify the state of the joint.
/// Zero (θ) corresponds to frames F and M being coincident and aligned.
/// The translation distance is defined positive when child body B translates
/// along the direction of â, and the rotation θ is defined to be positive
/// according to the right-hand-rule with the thumb aligned in the direction
/// of the `â_F` axis.
///
/// @tparam_default_scalar
template <typename T>
class ScrewJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ScrewJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  /// The name for this Joint type.
  static constexpr char kTypeName[] = "screw";

  /// Constructor to create a screw joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// translate and rotate as described in the class's documentation.
  /// This constructor signature creates a joint with the axis â set to the
  /// z-axis and no joint limits, i.e. the
  /// joint angular position, angular velocity and angular acceleration limits
  /// are the pair `(-∞, ∞)`.
  /// These can be set using the Joint methods set_position_limits(),
  /// set_velocity_limits() and set_acceleration_limits() in radians,
  /// radians/s, radians/s² units.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] screw_pitch
  ///   Amount of translation in meters occurring over a one full
  ///   screw revolution. It's domain is (-∞, ∞). When the screw pitch is
  ///   negative, positive rotation will result in translating towards the
  ///   negative direction of z-axis. When the screw pitch is zero, this joint
  ///   will behave like a revolute joint.
  /// @param[in] damping
  ///   Viscous damping coefficient, N⋅m⋅s/rad for
  ///   rotation, used to model losses within the joint. See documentation of
  ///   default_damping() for details on modelling of the damping torque.
  /// @throws std::exception if damping is negative.
  ScrewJoint(const std::string& name, const Frame<T>& frame_on_parent,
             const Frame<T>& frame_on_child, double screw_pitch, double damping)
      : ScrewJoint<T>(name, frame_on_parent, frame_on_child,
                      Vector3<double>::UnitZ(), screw_pitch, damping) {}

  /// Constructor to create a screw joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// translate and rotate as described in the class's documentation.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint angular position, angular velocity and angular acceleration limits
  /// are the pair `(-∞, ∞)`.
  /// These can be set using the Joint methods set_position_limits(),
  /// set_velocity_limits() and set_acceleration_limits() in radians,
  /// radians/s, radians/s² units.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the axis of motion for this joint. The
  ///   coordinates of `axis` expressed in frames F and M are the same at all
  ///   times, that is, `axis_F = axis_M`. In other words,
  ///   `axis_F` (or `axis_M`) is the eigenvector of `R_FM` with eigenvalue
  ///   equal to one.
  ///   This vector can have any length, only the direction is used.
  /// @param[in] screw_pitch
  ///   Amount of translation in meters occurring over a one full
  ///   screw revolution. It's domain is (-∞, ∞). When the screw pitch is
  ///   negative, positive rotation will result in translating towards the
  ///   negative direction of â-axis. When the screw pitch is zero, this joint
  ///   will behave like a revolute joint.
  /// @param[in] damping
  ///   Viscous damping coefficient, N⋅m⋅s/rad for
  ///   rotation, used to model losses within the joint. See documentation of
  ///   default_damping() for details on modelling of the damping torque.
  /// @throws std::exception if the L2 norm of `axis` is less than the square
  /// root of machine epsilon.
  /// @throws std::exception if damping is negative.
  ScrewJoint(const std::string& name, const Frame<T>& frame_on_parent,
             const Frame<T>& frame_on_child, const Vector3<double>& axis,
             double screw_pitch, double damping);

  ~ScrewJoint() final;

  const std::string& type_name() const final;

  /// Returns the normalized axis of motion of `this` joint as a unit vector.
  /// Since the measures of this axis in either frame F or M are the same (see
  /// this class's documentation for frame definitions) then,
  /// `axis = axis_F = axis_M`.
  const Vector3<double>& screw_axis() const { return axis_; }

  /// Returns `this` joint's amount of translation in meters
  /// occurring over a one full revolution.
  double screw_pitch() const { return screw_pitch_; }

  /// Returns `this` joint's default damping constant N⋅m⋅s for the rotational
  /// degree. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω` i.e.
  ///  opposing motion, with ω the angular rate for `this` joint
  ///  (see get_angular_velocity()) and τ the torque on
  /// child body B expressed in frame F as t_B_F = τ⋅Fâ_F.
  double default_damping() const { return this->default_damping_vector()[0]; }

  /// @name Context-dependent value access
  /// @{

  /// Gets the translation of `this` joint from `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval z The translation of `this` joint stored in the `context` as (z).
  ///           See class documentation for details.
  T get_translation(const Context<T>& context) const {
    return get_mobilizer().get_translation(context);
  }

  /// Sets the `context` so that the translation of `this` joint equals to (z).
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] z The desired translation in meters to be stored in `context`
  ///              as (z). See class documentation for details.
  /// @returns a constant reference to `this` joint.
  const ScrewJoint<T>& set_translation(Context<T>* context, const T& z) const {
    get_mobilizer().SetTranslation(context, z);
    return *this;
  }

  /// Gets the angle θ of `this` joint from `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval theta The angle of `this` joint stored in the `context`. See class
  ///               documentation for details.
  const T& get_rotation(const systems::Context<T>& context) const {
    return get_mobilizer().get_angle(context);
  }

  /// Sets the `context` so that the angle θ of `this` joint equals `theta`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] theta The desired angle in radians to be stored in `context`.
  ///                  See class documentation for details.
  /// @returns a constant reference to `this` joint.
  const ScrewJoint<T>& set_rotation(systems::Context<T>* context,
                                    const T& theta) const {
    get_mobilizer().SetAngle(context, theta);
    return *this;
  }

  /// Gets the translational velocity vz, in meters per second, of `this`
  /// joint's Mo measured and expressed in frame F from `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval vz The translational velocity of `this` joint as stored in the
  ///            `context`.
  T get_translational_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_translation_rate(context);
  }

  /// Sets the translational velocity, in meters per second, of this `this`
  /// joint's Mo along frame F's â-axis to `vz`. The new
  /// translational velocity gets stored in `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] vz The desired translational velocity of `this` joint in meters
  ///               per second along F frame's â-axis.
  /// @returns a constant reference to `this` joint.
  const ScrewJoint<T>& set_translational_velocity(systems::Context<T>* context,
                                                  const T& vz) const {
    get_mobilizer().SetTranslationRate(context, vz);
    return *this;
  }

  /// Gets the rate of change, in radians per second, of `this` joint's angle
  /// θ from `context`.  See class documentation for the definition of this
  /// angle.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval theta_dot The rate of change of `this` joint's angle θ as
  ///                   stored in the `context`.
  const T& get_angular_velocity(const systems::Context<T>& context) const {
    return get_mobilizer().get_angular_rate(context);
  }

  /// Sets the rate of change, in radians per second, of `this` joint's angle
  /// θ (see class documentation) to `theta_dot`. The new rate of change gets
  /// stored in `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] theta_dot The desired rates of change of `this` joint's
  ///                      angle in radians per second.
  /// @returns a constant reference to `this` joint.
  const ScrewJoint<T>& set_angular_velocity(systems::Context<T>* context,
                                            const T& theta_dot) const {
    get_mobilizer().SetAngularRate(context, theta_dot);
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

  /// Gets the default position for `this` joint.
  /// @retval z The default position of `this` joint.
  double get_default_translation() const {
    return internal::GetScrewTranslationFromRotation(
        this->default_positions()[0], screw_pitch());
  }

  /// Sets the default translation of this joint. This will change
  /// the `default_rotation` too, which are not independent in this joint.
  /// @param[in] z The desired default translation of the joint
  /// @throws std::exception if pitch is very near zero.
  void set_default_translation(const double& z) {
    Vector1<double> state(
        internal::GetScrewRotationFromTranslation(z, screw_pitch()));
    this->set_default_positions(state);
  }

  /// Gets the default angle for `this` joint.
  /// @retval theta The default angle of `this` joint.
  double get_default_rotation() const { return this->default_positions()[0]; }

  /// Sets the default angle of this joint. This will change the
  /// `default_translation` too, because they are not independent in this joint.
  /// @param[in] theta The desired default angle of the joint
  void set_default_rotation(double theta) {
    Vector1<double> state(theta);
    this->set_default_positions(state);
  }

  /// Sets the random distribution that the angle of this
  /// joint will be randomly sampled from. See class documentation for details
  /// on the definition of the position and angle.
  void set_random_pose_distribution(
      const Vector1<symbolic::Expression>& theta) {
    get_mutable_mobilizer().set_random_position_distribution(theta);
  }

 private:
  /* Joint<T> override called through public NVI, Joint::AddInForce().
   Therefore arguments were already checked to be valid.
   For a ScrewJoint, we must always have `joint_dof = 0` since there is
   one degree of freedom (num_velocities() == 1). `joint_tau` is the torque
   about the â-axis of the parent frame F if `joint_dof = 0`.
   The force is applied to the body declared as child (according to the
   screw joint's constructor) at the origin of the child frame M. The force
   is defined to be positive in the direction of the selected axis and the
   torque is defined to be positive according to the right hand rule about
   the selected axis. That is, a positive force causes a positive
   translational acceleration and a positive torque causes a positive angular
   acceleration (of the child body frame) [assuming a positive screw pitch]. */
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const final {
    DRAKE_DEMAND(joint_dof < 1);
    Eigen::Ref<VectorX<T>> tau_mob =
        get_mobilizer().get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  /*  Joint<T> override called through public NVI, Joint::AddInDamping().
    Therefore arguments were already checked to be valid.
    This method adds into `forces` a dissipative force according to the
    viscous law `f = -d⋅v`, with d the damping coefficient (see
    default_damping()). */
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final {
    Eigen::Ref<VectorX<T>> tau =
        get_mobilizer().get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    const T& v_angular = get_angular_velocity(context);
    tau[0] -= this->GetDamping(context) * v_angular;
  }

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
    return get_rotation(context);
  }

  const T& DoGetOneVelocity(const systems::Context<T>& context) const final {
    return get_angular_velocity(context);
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

  // Make ScrewJoint templated on every other scalar type a friend of
  // ScrewJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of ScrewJoint<T>.
  template <typename>
  friend class ScrewJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::ScrewMobilizer<T>& get_mobilizer() const {
    return this->template get_mobilizer_downcast<internal::ScrewMobilizer>();
  }

  internal::ScrewMobilizer<T>& get_mutable_mobilizer() {
    return this
        ->template get_mutable_mobilizer_downcast<internal::ScrewMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  Vector3<double> axis_;

  // The amount of translation in meters occurring over a one full revolution.
  double screw_pitch_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ScrewJoint);
