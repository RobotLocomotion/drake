#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/planar_mobilizer.h"

namespace drake {
namespace multibody {

/// This joint models a planar joint allowing two bodies to translate and rotate
/// relative to one another in a plane with three degrees of freedom.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation), this
/// joint allows frame M to translate within the x-y plane of frame F and to
/// rotate about the z-axis, with M's z-axis Mz and F's z-axis Fz coincident at
/// all times. The translations along the x- and y-axes of F, the rotation about
/// the z-axis and their rates specify the state of the joint.
/// Zero (x, y, θ) corresponds to frames F and M being coincident and aligned.
/// Translation (x, y) is defined to be positive in the direction of the
/// respective axes and the rotation θ is defined to be positive according to
/// the right-hand-rule with the thumb aligned in the direction of frame F's
/// z-axis.
///
/// @tparam_default_scalar
template <typename T>
class PlanarJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PlanarJoint)

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  /// The name for this Joint type.
  static constexpr char kTypeName[] = "planar";

  /// Constructor to create a planar joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// translate and rotate as described in the class's documentation.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint position, velocity and acceleration limits are the pair `(-∞, ∞)`.
  /// These can be set using the Joint methods set_position_limits(),
  /// set_velocity_limits() and set_acceleration_limits().
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅s/m for translation and N⋅m⋅s for
  ///   rotation, used to model losses within the joint. See documentation of
  ///   damping() for details on modelling of the damping force and torque.
  /// @throws std::exception if any element of damping is negative.
  PlanarJoint(const std::string& name, const Frame<T>& frame_on_parent,
              const Frame<T>& frame_on_child, Vector3<double> damping)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS((damping.array() >= 0).all());
    damping_ = damping;
  }

  const std::string& type_name() const final {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns `this` joint's damping constant in N⋅s/m for the translational
  /// degrees and N⋅m⋅s for the rotational degree. The damping force (in N) is
  /// modeled as `fᵢ = -dampingᵢ⋅vᵢ, i = 1, 2` i.e. opposing motion, with vᵢ
  /// the translation rates along the i-th axis for `this` joint (see
  /// get_translational_velocity()) and fᵢ the force on child body B at Mo and
  /// expressed in F. That is, f_BMo_F = (f₁, f₂). The damping torque (in N⋅m)
  /// is modeled as `τ = -damping₃⋅ω` i.e. opposing motion, with ω the angular
  /// rate for `this` joint (see get_angular_velocity()) and τ the torque on
  /// child body B expressed in frame F as t_B_F = τ⋅Fz_F.
  Vector3<double> damping() const { return damping_; }

  /// @name Context-dependent value access
  /// @{

  /// Gets the position of `this` joint from `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval p_FoMo_F The position of `this` joint stored in the `context`
  ///                  ordered as (x, y). See class documentation for details.
  Vector2<T> get_translation(const Context<T>& context) const {
    return get_mobilizer()->get_translations(context);
  }

  /// Sets the `context` so that the position of `this` joint equals `p_FoMo_F`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] p_FoMo_F The desired position in meters to be stored in
  ///                     `context` ordered as (x, y). See class documentation
  ///                     for details.
  /// @returns a constant reference to `this` joint.
  const PlanarJoint<T>& set_translation(Context<T>* context,
                                        const Vector2<T>& p_FoMo_F) const {
    get_mobilizer()->set_translations(context, p_FoMo_F);
    return *this;
  }

  /// Gets the angle θ of `this` joint from `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval theta The angle of `this` joint stored in the `context`. See class
  ///               documentation for details.
  const T& get_rotation(const systems::Context<T>& context) const {
    return get_mobilizer()->get_angle(context);
  }

  /// Sets the `context` so that the angle θ of `this` joint equals `theta`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] theta The desired angle in radians to be stored in `context`.
  ///                  See class documentation for details.
  /// @returns a constant reference to `this` joint.
  const PlanarJoint<T>& set_rotation(systems::Context<T>* context,
                                     const T& theta) const {
    get_mobilizer()->set_angle(context, theta);
    return *this;
  }

  /// Sets the `context` so that the position of `this` joint equals `p_FoMo_F`
  /// and its angle equals `theta`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] p_FoMo_F The desired position in meters to be stored in
  ///                     `context` ordered as (x, y). See class documentation
  ///                     for details.
  /// @param[in] theta The desired angle in radians to be stored in `context`.
  ///                  See class documentation for details.
  /// @returns a constant reference to `this` joint.
  const PlanarJoint<T>& set_pose(systems::Context<T>* context,
                                 const Vector2<T>& p_FoMo_F,
                                 const T& theta) const {
    get_mobilizer()->set_translations(context, p_FoMo_F);
    get_mobilizer()->set_angle(context, theta);
    return *this;
  }

  /// Gets the translational velocity v_FoMo_F, in meters per second, of `this`
  /// joint's Mo measured and expressed in frame F from `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @retval v_FoMo_F The translational velocity of `this` joint as stored in
  ///                  the `context`.
  Vector2<T> get_translational_velocity(
      const systems::Context<T>& context) const {
    return get_mobilizer()->get_translation_rates(context);
  }

  /// Sets the translational velocity, in meters per second, of this `this`
  /// joint's Mo measured and expressed in frame F  to `v_FoMo_F`. The new
  /// translational velocity gets stored in `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] v_FoMo_F The desired translational velocity of `this` joint in
  ///                     meters per second.
  /// @returns a constant reference to `this` joint.
  const PlanarJoint<T>& set_translational_velocity(
      systems::Context<T>* context, const Vector2<T>& v_FoMo_F) const {
    get_mobilizer()->set_translation_rates(context, v_FoMo_F);
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
    return get_mobilizer()->get_angular_rate(context);
  }

  /// Sets the rate of change, in radians per second, of `this` joint's angle
  /// θ (see class documentation) to `theta_dot`. The new rate of change gets
  /// stored in `context`.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] theta_dot The desired rates of change of `this` joint's
  ///                      angle in radians per second.
  /// @returns a constant reference to `this` joint.
  const PlanarJoint<T>& set_angular_velocity(systems::Context<T>* context,
                                             const T& theta_dot) const {
    get_mobilizer()->set_angular_rate(context, theta_dot);
    return *this;
  }

  /// @}

  /// Gets the default position for `this` joint.
  /// @retval p_FoMo_F The default position of `this` joint.
  Vector2<double> get_default_translation() const {
    return this->default_positions().head(2);
  }

  /// Sets the default position of this joint.
  /// @param[in] p_FoMo_F The desired default position of the joint
  void set_default_translation(const Vector2<double>& p_FoMo_F) {
    Vector3<double> state(p_FoMo_F[0], p_FoMo_F[1],
                          this->default_positions()[2]);
    this->set_default_positions(state);
  }

  /// Gets the default angle for `this` joint.
  /// @retval theta The default angle of `this` joint.
  double get_default_rotation() const { return this->default_positions()[2]; }

  /// Sets the default angle of this joint.
  /// @param[in] theta The desired default angle of the joint
  void set_default_rotation(double theta) {
    Vector3<double> state = this->default_positions();
    state[2] = theta;
    this->set_default_positions(state);
  }

  /// Sets the default position and angle of this joint.
  /// @param[in] p_FoMo_F The desired default position of the joint
  /// @param[in] theta The desired default angle of the joint
  void set_default_pose(const Vector2<double>& p_FoMo_F, double theta) {
    Vector3<double> state(p_FoMo_F[0], p_FoMo_F[1], theta);
    this->set_default_positions(state);
  }

  /// Sets the random distribution that the position and angle of this
  /// joint will be randomly sampled from. See class documentation for details
  /// on the definition of the position and angle.
  void set_random_pose_distribution(
      const Vector2<symbolic::Expression>& p_FoMo_F,
      const symbolic::Expression& theta) {
    Vector3<symbolic::Expression> state(p_FoMo_F[0], p_FoMo_F[1], theta);
    get_mutable_mobilizer()->set_random_position_distribution(state);
  }

 private:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a PlanarJoint, we must always have `joint_dof < 3` since there are
  /// three degrees of freedom (num_velocities() == 3). `joint_tau` is the force
  /// applied along the x-axis of the parent  frame F if `joint_dof = 0`, the
  /// force applied along the y-axis of the parent frame F if `joint_dof = 1`,
  /// or the torque about the z-axis of the parent frame F if `joint_dof = 2`.
  /// The force is applied to the body declared as child (according to the
  /// planar joint's constructor) at the origin of the child frame M. The force
  /// is defined to be positive in the direction of the selected axis and the
  /// torque is defined to be positive according to the right hand rule about
  /// the selected axis. That is, a positive force causes a positive
  /// translational acceleration and a positive torque causes a positive angular
  /// acceleration (of the child body frame).
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const final {
    DRAKE_DEMAND(joint_dof < 3);
    Eigen::Ref<VectorX<T>> tau_mob =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

  /// Joint<T> override called through public NVI, Joint::AddInDamping().
  /// Therefore arguments were already checked to be valid.
  /// This method adds into `forces` a dissipative force according to the
  /// viscous law `f = -d⋅v`, with d the damping coefficient (see damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final {
    Eigen::Ref<VectorX<T>> tau =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    const Vector2<T>& v_translation = get_translational_velocity(context);
    const T& v_angular = get_angular_velocity(context);
    const Vector3<double> damping_coeff = damping();
    tau[0] -= damping_coeff[0] * v_translation[0];
    tau[1] -= damping_coeff[1] * v_translation[1];
    tau[2] -= damping_coeff[2] * v_angular;
  }

  int do_get_velocity_start() const final {
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const final { return 3; }

  int do_get_position_start() const final {
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const final { return 3; }

  std::string do_get_position_suffix(int index) const override {
    return get_mobilizer()->position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const override {
    return get_mobilizer()->velocity_suffix(index);
  }

  void do_set_default_positions(
      const VectorX<double>& default_positions) final {
    if (this->has_implementation()) {
      get_mutable_mobilizer()->set_default_position(default_positions);
    }
  }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint()
      const final;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const final;

  // Make PlanarJoint templated on every other scalar type a friend of
  // PlanarJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of PlanarJoint<T>.
  template <typename>
  friend class PlanarJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::PlanarMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::PlanarMobilizer<T>* mobilizer =
        dynamic_cast<const internal::PlanarMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::PlanarMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::PlanarMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This joint's damping constant in N⋅s/m for translation and N⋅m⋅s for
  // rotation.
  Vector3<double> damping_;
};

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PlanarJoint)
