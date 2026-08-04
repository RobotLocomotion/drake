#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/joint.h"
#include "drake/multibody/tree/multibody_forces.h"
#include "drake/multibody/tree/prismatic_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to translate relative to one another along a
/// common axis.
/// That is, given a frame Jp attached to the parent body P and a frame Jc
/// attached to the child body C (see the Joint class's documentation),
/// this Joint allows frames Jp and Jc to translate with respect to each other
/// along an axis â. The translation distance is defined positive when child
/// body C translates along the direction of â. Axis vector â is constant and
/// has the same components in both frames Jp and Jc, that is, `â_Jp = â_Jc`.
///
/// @tparam_default_scalar
template <typename T>
class PrismaticJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticJoint);

  template <typename Scalar>
  using Context = systems::Context<Scalar>;

  static const char kTypeName[];

  /// Constructor to create a prismatic joint between two bodies so that
  /// frame Jp attached to the parent body P and frame Jc attached to the child
  /// body C, translate relatively to one another along a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and translation distance.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameter `axis` is:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the translation axis for this joint. Given
  ///   that frame Jc only translates with respect to Jp and there is no
  ///   relative rotation, the components of `axis` in either frame Jp or Jc
  ///   are exactly the same, that is, `axis_Jp = axis_Jc`. This vector can have
  ///   any length, only the direction is used.
  /// @param[in] pos_lower_limit
  ///   Lower position limit, in meters, for the translation coordinate
  ///   (see get_translation()).
  /// @param[in] pos_upper_limit
  ///   Upper position limit, in meters, for the translation coordinate
  ///   (see get_translation()).
  /// @param[in] damping
  ///   Viscous damping coefficient, in N⋅s/m, used to model losses within the
  ///   joint. The damping force (in N) is modeled as `f = -damping⋅v`, i.e.
  ///   opposing motion, with v the translational speed for `this` joint (see
  ///   get_translation_rate()).
  /// @throws std::exception if the L2 norm of `axis` is less than the square
  /// root of machine epsilon.
  /// @throws std::exception if damping is negative.
  /// @throws std::exception if pos_lower_limit > pos_upper_limit.
  PrismaticJoint(
      const std::string& name, const Frame<T>& frame_on_parent,
      const Frame<T>& frame_on_child, const Vector3<double>& axis,
      double pos_lower_limit = -std::numeric_limits<double>::infinity(),
      double pos_upper_limit = std::numeric_limits<double>::infinity(),
      double damping = 0);

  ~PrismaticJoint() final;

  const std::string& type_name() const final;

  /// Returns the axis of translation for `this` joint as a unit vector.
  /// Since the components of this axis in either frame Jp or Jc are the same
  /// (see this class's documentation for frame definitions) then,
  /// `axis = axis_Jp = axis_Jc`.
  const Vector3<double>& translation_axis() const { return axis_; }

  /// Returns `this` joint's default damping constant in N⋅s/m.
  double default_damping() const { return this->default_damping_vector()[0]; }

  /// Sets the default value of viscous damping for this joint, in N⋅s/m.
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

  /// Returns the position lower limit for `this` joint in meters.
  double position_lower_limit() const {
    return this->position_lower_limits()[0];
  }

  /// Returns the position upper limit for `this` joint in meters.
  double position_upper_limit() const {
    return this->position_upper_limits()[0];
  }

  /// Returns the velocity lower limit for `this` joint in meters per second.
  double velocity_lower_limit() const {
    return this->velocity_lower_limits()[0];
  }

  /// Returns the velocity upper limit for `this` joint in meters per second.
  double velocity_upper_limit() const {
    return this->velocity_upper_limits()[0];
  }

  /// Returns the acceleration lower limit for `this` joint in meters per second
  /// squared.
  double acceleration_lower_limit() const {
    return this->acceleration_lower_limits()[0];
  }

  /// Returns the acceleration upper limit for `this` joint in meters per second
  /// squared.
  double acceleration_upper_limit() const {
    return this->acceleration_upper_limits()[0];
  }

  /// @name Context-dependent value access
  /// @{

  /// Gets the translation distance of `this` mobilizer from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The translation coordinate of `this` joint read from `context`.
  const T& get_translation(const Context<T>& context) const {
    return get_mobilizer().get_translation(context);
  }

  /// Sets `context` so that the generalized coordinate corresponding to the
  /// translation distance of `this` joint equals `translation`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] translation
  ///   The desired translation in meters to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const PrismaticJoint<T>& set_translation(Context<T>* context,
                                           const T& translation) const {
    get_mobilizer().SetTranslation(context, translation);
    return *this;
  }

  /// Gets the rate of change, in meters per second, of `this` joint's
  /// translation distance (see get_translation()) from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The rate of change of `this` joint's translation read from
  /// `context`.
  const T& get_translation_rate(const Context<T>& context) const {
    return get_mobilizer().get_translation_rate(context);
  }

  /// Sets the rate of change, in meters per second, of `this` joint's
  /// translation distance to `translation_dot`. The new rate of change
  /// `translation_dot` gets stored in `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] translation_dot
  ///   The desired rate of change of `this` joints's translation in meters per
  ///   second.
  /// @returns a constant reference to `this` joint.
  const PrismaticJoint<T>& set_translation_rate(
      Context<T>* context, const T& translation_dot) const {
    get_mobilizer().SetTranslationRate(context, translation_dot);
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

  /// Gets the default translation. Wrapper for the more general
  /// `Joint::default_positions()`.
  /// @returns The default translation of `this` stored in `default_positions_`.
  double get_default_translation() const {
    return this->default_positions()[0];
  }

  /// Sets the `default_positions` of this joint (in this case a single
  /// translation)
  /// @param[in] translation
  ///   The desired default translation of the joint
  void set_default_translation(double translation) {
    this->set_default_positions(Vector1d{translation});
  }

  void set_random_translation_distribution(
      const symbolic::Expression& translation) {
    get_mutable_mobilizer().set_random_position_distribution(
        Vector1<symbolic::Expression>{translation});
  }

  /// Adds into `multibody_forces` a given `force`, in Newtons, for `this` joint
  /// that is to be applied along the joint's axis. The force is defined to be
  /// positive in the direction along this joint's axis.
  /// That is, a positive force causes a positive translational acceleration
  /// along the joint's axis.
  void AddInForce(const systems::Context<T>& context, const T& force,
                  MultibodyForces<T>* multibody_forces) const {
    DRAKE_DEMAND(multibody_forces != nullptr);
    DRAKE_DEMAND(this->has_parent_tree());
    DRAKE_DEMAND(
        multibody_forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    this->AddInOneForce(context, 0, force, multibody_forces);
  }

 protected:
  /// Joint<T> virtual override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a %PrismaticJoint, we must always have `joint_dof = 0` since there is
  /// only a single degree of freedom (num_velocities() == 1). `joint_tau` is
  /// the linear force applied along the joint's axis, on the body declared as
  /// child (according to the prismatic joint's constructor) at the origin of
  /// the child frame (which is coincident with the origin of the parent frame
  /// at all times).
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
  /// This method adds into `forces` a dissipative force according to the
  /// viscous law `f = -d⋅v`, with d the damping coefficient (see
  /// default_damping()).
  void DoAddInDamping(const systems::Context<T>& context,
                      MultibodyForces<T>* forces) const final {
    const T damping_force =
        -this->GetDamping(context) * get_translation_rate(context);
    AddInForce(context, damping_force, forces);
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
    return get_translation(context);
  }

  const T& DoGetOneVelocity(const systems::Context<T>& context) const final {
    return get_translation_rate(context);
  }

  // Joint<T> finals:
  std::unique_ptr<internal::Mobilizer<T>> MakeMobilizerForJoint(
      const internal::SpanningForest::Mobod& mobod,
      internal::MultibodyTree<T>*) const final;

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const internal::MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const internal::MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Joint<symbolic::Expression>> DoCloneToScalar(
      const internal::MultibodyTree<symbolic::Expression>&) const final;

  std::unique_ptr<Joint<T>> DoShallowClone() const final;

  // Make PrismaticJoint templated on every other scalar type a friend of
  // PrismaticJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of PrismaticJoint<T>.
  template <typename>
  friend class PrismaticJoint;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const internal::PrismaticMobilizer<T>& get_mobilizer() const {
    return this
        ->template get_mobilizer_downcast<internal::PrismaticMobilizer>();
  }

  internal::PrismaticMobilizer<T>& get_mutable_mobilizer() {
    return this->template get_mutable_mobilizer_downcast<
        internal::PrismaticMobilizer>();
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const internal::MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  // It is a unit vector.
  Vector3<double> axis_;
};

template <typename T>
const char PrismaticJoint<T>::kTypeName[] = "prismatic";

}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticJoint);
