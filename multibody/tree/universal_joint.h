#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/universal_mobilizer.h"

namespace drake {
namespace multibody {

/// This joint models a universal joint allowing two bodies to rotate relative
/// to one another with two degrees of freedom.
/// A universal joint can be thought of as a mechanism consisting of three
/// bodies; the parent body P, an intermediate cross-shaped body I, and the
/// child body B. In a physical universal joint, body I has a significantly
/// smaller mass than P or B. This universal joint model corresponds to the
/// mathematical limit of having a body I of negligible mass. Given a frame F
/// attached to the parent body P and a frame M attached to the child body B
/// (see the Joint class's documentation), the orientation of M in F can then
/// naturally be defined as follows using a body fixed rotation sequence. A
/// first rotation of θ₁ about Fx defines the orientation R_FI of the
/// intermediate frame I attached to body I (notice that by definition Ix = Fx
/// at all times). A second rotation of θ₂ about Iy defines the orientation R_IM
/// of frame M (notice that by definition My = Iy at all times). Mathematically,
/// the orientation of frame M in F is given by <pre>
///   R_FM(q) = R_FI(θ₁) * R_IM(θ₂)
/// </pre>
/// No translational motion of M in F is allowed and the origins, `Mo` and `Fo`,
/// of frames M and F respectively remain coincident. The angles of rotation
/// about F's x-axis and M's y-axis, along with their rates, specify the state
/// of the joint.
/// Zero θ₁, θ₂ angles corresponds to frames F, I, and M being coincident.
/// Angles (θ₁, θ₂) are defined to be positive according to the
/// right-hand-rule with the thumb aligned in the direction of their
/// respective axes.
///
/// @tparam_default_scalar
template <typename T>
class UniversalJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(UniversalJoint)

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  /// The name for this Joint type.  It resolves to "universal".
  static const char kTypeName[];

  /// Constructor to create a universal joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// rotate as described in the class's documentation. See class documentation
  /// for details on the angles defining orientation.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint position, velocity and acceleration limits are the pair `(-∞, ∞)`.
  /// These can be set using the Joint methods set_position_limits(),
  /// set_velocity_limits() and set_acceleration_limits().
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. See documentation of damping() for details on modelling of the
  ///   damping torque.
  /// @throws std::exception if damping is negative.
  UniversalJoint(const std::string& name, const Frame<T>& frame_on_parent,
                 const Frame<T>& frame_on_child, double damping = 0)
      : Joint<T>(name, frame_on_parent, frame_on_child,
                 VectorX<double>::Constant(
                     2, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     2, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     2, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     2, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     2, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     2, std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS(damping >= 0);
    damping_ = damping;
  }

  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns `this` joint's damping constant in N⋅m⋅s. The damping torque
  /// (in N⋅m) is modeled as `τᵢ = -damping⋅ωᵢ, i = 1, 2` i.e. opposing motion,
  /// with ωᵢ the angular rates about the i-th axis for `this` joint (see
  /// get_angular_rates())and τᵢ the torque on child body B about the same i-th
  /// axis.
  double damping() const { return damping_; }

  /// @name Context-dependent value access
  /// @{

  /// Gets the rotation angles of `this` joint from `context`. See class
  /// documentation for the definition of these angles.
  ///
  /// @param[in] context The context of the model this joint belongs to.
  /// @returns The angle coordinates of `this` joint stored in the `context`
  ///          ordered as (θ₁, θ₂).
  Vector2<T> get_angles(const Context<T>& context) const {
    return get_mobilizer()->get_angles(context);
  }

  /// Sets the `context` so that the generalized coordinates corresponding to
  /// the rotation angles of `this` joint equals `angles`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] angles The desired angles in radians to be stored in `context`
  ///                   ordered as (θ₁, θ₂). See class documentation for
  ///                   details.
  /// @returns a constant reference to `this` joint.
  const UniversalJoint<T>& set_angles(Context<T>* context,
                                      const Vector2<T>& angles) const {
    get_mobilizer()->set_angles(context, angles);
    return *this;
  }

  /// Gets the rates of change, in radians per second, of `this` joint's
  /// angles (see class documentation) from `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @returns The rates of change of `this` joint's angles as stored in the
  ///          `context`.
  Vector2<T> get_angular_rates(const systems::Context<T>& context) const {
    return get_mobilizer()->get_angular_rates(context);
  }

  /// Sets the rates of change, in radians per second, of this `this` joint's
  /// angles (see class documentation) to `theta_dot`. The new rates of change
  /// get stored in `context`.
  /// @param[in] context The context of the model this joint belongs to.
  /// @param[in] theta_dot The desired rates of change of `this` joints's angles
  ///                      in radians per second.
  /// @returns a constant reference to `this` joint.
  const UniversalJoint<T>& set_angular_rates(
      systems::Context<T>* context, const Vector2<T>& theta_dot) const {
    get_mobilizer()->set_angular_rates(context, theta_dot);
    return *this;
  }

  /// @}

  /// Gets the default angles for `this` joint. Wrapper for the more general
  /// `Joint::default_positions()`.
  /// @returns The default angles of `this` stored in `default_positions_`
  Vector2<double> get_default_angles() const {
    return this->default_positions();
  }

  /// Sets the default angles of this joint.
  /// @param[in] angles The desired default angles of the joint
  void set_default_angles(const Vector2<double>& angles) {
    this->set_default_positions(angles);
  }

  /// Sets the random distribution that angles of this joint will be randomly
  /// sampled from. See class documentation for details on the definition of the
  /// angles.
  void set_random_angles_distribution(
      const Vector2<symbolic::Expression>& angles) {
    get_mutable_mobilizer()->set_random_position_distribution(
        Vector2<symbolic::Expression>{angles});
  }

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a UniversalJoint, we must always have `joint_dof < 2` since there are
  /// two degrees of freedom (num_velocities() == 2). `joint_tau` is the torque
  /// applied about the axis specified by `joint_dof`, the x-axis of the parent
  /// frame F if `joint_dof = 0` or the y-axis of the child frame M if
  /// `joint_dof = 1`.  The torque is applied to the body declared as child
  /// (according to the universal joint's constructor) at the origin of the
  /// child frame M (which is coincident with the origin of the parent frame F
  /// at all times). The torque is defined to be positive according to
  /// the right-hand-rule with the thumb aligned in the direction of the
  /// selected axis. That is, a positive torque causes a positive rotational
  /// acceleration (of the child body frame).
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const override {
    DRAKE_DEMAND(joint_dof < 2);
    Eigen::Ref<VectorX<T>> tau_mob =
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
    Eigen::Ref<VectorX<T>> tau =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    const Vector2<T>& theta_dot = get_angular_rates(context);
    tau = -damping() * theta_dot;
  }

 private:
  int do_get_velocity_start() const override {
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const override { return 2; }

  int do_get_position_start() const override {
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const override { return 2; }

  std::string do_get_position_suffix(int index) const override {
    return get_mobilizer()->position_suffix(index);
  }

  std::string do_get_velocity_suffix(int index) const override {
    return get_mobilizer()->velocity_suffix(index);
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

  // Make UniversalJoint templated on every other scalar type a friend of
  // UniversalJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of UniversalJoint<T>.
  template <typename>
  friend class UniversalJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::UniversalMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::UniversalMobilizer<T>* mobilizer =
        dynamic_cast<const internal::UniversalMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::UniversalMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::UniversalMobilizer<T>*>(
        this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This joint's damping constant in N⋅m⋅s.
  double damping_{0};
};

template <typename T>
const char UniversalJoint<T>::kTypeName[] = "universal";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::UniversalJoint)
