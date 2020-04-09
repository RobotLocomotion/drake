#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/space_xyz_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to rotate freely relative to one another.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frame M to rotate freely with respect to F. The
/// orientation of B relative to P is parameterized with space `x-y-z` Euler
/// angles.
///
/// @tparam_default_scalar
template <typename T>
class GimbalJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(GimbalJoint)

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  /// Constructor to create a gimbal joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// rotate freely relative to one another. See this class's documentation
  /// for further details on the definition of these frames and rotation angles.
  /// This constructor signature creates a joint with no joint limits, i.e. the
  /// joint position, velocity and acceleration limits are the pair `(-∞, ∞)`.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e.
  ///   opposing motion, with ω the angular rate for `this` joint (see
  ///   get_angular_rate()).
  /// @throws std::exception if damping is negative.
  GimbalJoint(const std::string& name, const Frame<T>& frame_on_parent,
              const Frame<T>& frame_on_child, double damping = 0)
      : GimbalJoint<T>(
            name, frame_on_parent, frame_on_child,
            Vector3<double>::Constant(-std::numeric_limits<double>::infinity()),
            Vector3<double>::Constant(std::numeric_limits<double>::infinity()),
            damping) {}

  /// Constructor to create a gimbal joint between two bodies so that frame F
  /// attached to the parent body P and frame M attached to the child body B
  /// rotate freely relative to one another. See this class's documentation
  /// for further details on the definition of these frames and rotation angles.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameters are:
  /// @param[in] pos_lower_limit
  ///   Lower position limit, in radians, for the rotation coordinates
  ///   (see get_angle()).
  /// @param[in] pos_upper_limit
  ///   Upper position limit, in radians, for the rotation coordinates
  ///   (see get_angle()).
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅m⋅s, used to model losses within the
  ///   joint. The damping torque (in N⋅m) is modeled as `τ = -damping⋅ω`, i.e.
  ///   opposing motion, with ω the angular rate for `this` joint (see
  ///   get_angular_rate()).
  /// @throws std::exception if damping is negative.
  /// @throws std::exception if pos_lower_limit > pos_upper_limit.
  GimbalJoint(const std::string& name, const Frame<T>& frame_on_parent,
              const Frame<T>& frame_on_child,
              const Vector3<double>& pos_lower_limit,
              const Vector3<double>& pos_upper_limit,
                double damping = 0)
      : Joint<T>(name, frame_on_parent, frame_on_child, pos_lower_limit,
                 pos_upper_limit,
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, -std::numeric_limits<double>::infinity()),
                 VectorX<double>::Constant(
                     3, std::numeric_limits<double>::infinity())) {
    DRAKE_THROW_UNLESS(damping >= 0);
    damping_ = damping;
  }

  const std::string& type_name() const override {
    static const never_destroyed<std::string> name{kTypeName};
    return name.access();
  }

  /// Returns `this` joint's damping constant in N⋅m⋅s.
  double damping() const { return damping_; }

  /// @name Context-dependent value access
  /// @{

  /// Gets the rotation angles of `this` mobilizer from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The angle coordinates of `this` joint stored in the `context`.
  Vector3<T> get_angles(const Context<T>& context) const {
    return get_mobilizer()->get_angles(context);
  }

  /// Sets the `context` so that the generalized coordinates corresponding to
  /// the rotation angles of `this` joint equals `angles`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] angles
  ///   The desired angles in radians to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const GimbalJoint<T>& set_angles(Context<T>* context,
                                   const Vector3<T>& angles) const {
    get_mobilizer()->set_angles(context, angles);
    return *this;
  }

  void set_default_angles(const Vector3<double>& angles) {
    get_mutable_mobilizer()->set_default_position(angles);
  }

  void set_random_angles_distribution(
      const Vector3<symbolic::Expression>& angles) {
    get_mutable_mobilizer()->set_random_position_distribution(
        Vector3<symbolic::Expression>{angles});
  }

  /// Retrieves from `context` the angular velocity `w_FM` of the child body's
  /// frame M in the parent body's frame F, expressed in F.
  ///
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @retval w_FM
  ///   A vector in ℝ³ with the angular velocity of the child body's
  ///   frame M in the parent body's frame F, expressed in F.
  Vector3<T> get_angular_velocity(const systems::Context<T>& context) const {
    return get_mobilizer()->get_angular_velocity(context);
  }

  /// Sets in `context` the state for `this` joint so that the angular velocity
  /// of the child body's frame M in the parent body's frame F is `w_FM`.
  /// @param[out] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] w_FM
  ///   A vector in ℝ³ with the desired angular velocity of the child body's
  ///   frame M in the parent body's frame F, expressed in F.
  /// @returns a constant reference to `this` joint.
  const GimbalJoint<T>& set_angular_velocity(systems::Context<T>* context,
                                             const Vector3<T>& w_FM) const {
    get_mobilizer()->set_angular_velocity(context, w_FM);
    return *this;
  }

  /// @}

 protected:
  /// Joint<T> override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a %GimbalJoint, `joint_dof` selects the desired axis about which to
  /// apply the torque (see class documentation for ordering). `joint_tau` is
  /// the torque applied about the selected axis, on the body declared as child
  /// (according to the gimbal joint's constructor) at the origin of the child
  /// frame (which is coincident with the origin of the parent frame at all
  /// times). The torque is defined to be positive according to
  /// the right-hand-rule with the thumb aligned in the direction of the
  /// selected axis. That is, a positive torque causes a positive rotational
  /// acceleration (of the child body frame) according to the right-hand-rule
  /// around the axis.
  void DoAddInOneForce(const systems::Context<T>&, int joint_dof,
                       const T& joint_tau,
                       MultibodyForces<T>* forces) const override {
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
    const Vector3<T> damping_torque =
        -this->damping() * get_angular_velocity(context);
    this->AddInOneForce(context, 0, damping_torque[0], forces);
    this->AddInOneForce(context, 1, damping_torque[1], forces);
    this->AddInOneForce(context, 2, damping_torque[2], forces);
  }

 private:
  int do_get_velocity_start() const override {
    return get_mobilizer()->velocity_start_in_v();
  }

  int do_get_num_velocities() const override { return 3; }

  int do_get_position_start() const override {
    return get_mobilizer()->position_start_in_q();
  }

  int do_get_num_positions() const override { return 3; }

  // Joint<T> overrides:
  std::unique_ptr<typename Joint<T>::BluePrint> MakeImplementationBlueprint()
      const override;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const override;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const override;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const override;

  // Make GimbalJoint templated on every other scalar type a friend of
  // GimbalJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of GimbalJoint<T>.
  template <typename>
  friend class GimbalJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::SpaceXYZMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const internal::SpaceXYZMobilizer<T>* mobilizer =
        dynamic_cast<const internal::SpaceXYZMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  internal::SpaceXYZMobilizer<T>* get_mutable_mobilizer() {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    auto* mobilizer = dynamic_cast<internal::SpaceXYZMobilizer<T>*>(
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
const char GimbalJoint<T>::kTypeName[] = "gimbal";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::GimbalJoint)
