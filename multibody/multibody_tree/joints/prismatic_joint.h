#pragma once

#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "drake/common/drake_copyable.h"
#include "drake/multibody/multibody_tree/joints/joint.h"
#include "drake/multibody/multibody_tree/multibody_forces.h"
#include "drake/multibody/multibody_tree/prismatic_mobilizer.h"

namespace drake {
namespace multibody {

/// This Joint allows two bodies to translate relative to one another along a
/// common axis.
/// That is, given a frame F attached to the parent body P and a frame M
/// attached to the child body B (see the Joint class's documentation),
/// this Joint allows frames F and M to translate with respect to each other
/// along an axis â. The translation distance is defined positive when child
/// body B translates along the direction of â.
/// Axis â is constant and has the same measures in both frames F and M, that
/// is, `â_F = â_M`.
///
/// @tparam T The scalar type. Must be a valid Eigen scalar.
///
/// Instantiated templates for the following kinds of T's are provided:
/// - double
/// - AutoDiffXd
///
/// They are already available to link against in the containing library.
/// No other values for T are currently supported.
template <typename T>
class PrismaticJoint final : public Joint<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PrismaticJoint)

  template<typename Scalar>
  using Context = systems::Context<Scalar>;

  /// Constructor to create a prismatic joint between two bodies so that
  /// frame F attached to the parent body P and frame M attached to the child
  /// body B, translate relatively to one another along a common axis. See this
  /// class's documentation for further details on the definition of these
  /// frames and translation distance.
  /// The first three arguments to this constructor are those of the Joint class
  /// constructor. See the Joint class's documentation for details.
  /// The additional parameter `axis` is:
  /// @param[in] axis
  ///   A vector in ℝ³ specifying the translation axis for this joint. Given
  ///   that frame M only translates with respect to F and there is no relative
  ///   rotation, the measures of `axis` in either frame F or M
  ///   are exactly the same, that is, `axis_F = axis_M`.
  ///   This vector can have any length, only the direction is used.
  /// @throws std::exception if the L2 norm of `axis` is less than the square
  /// root of machine epsilon.
  PrismaticJoint(const std::string& name,
                const Frame<T>& frame_on_parent, const Frame<T>& frame_on_child,
                const Vector3<double>& axis) :
      Joint<T>(name, frame_on_parent, frame_on_child) {
    const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
    DRAKE_THROW_UNLESS(!axis.isZero(kEpsilon));
    axis_ = axis.normalized();
  }

  /// Returns the axis of translation for `this` joint as a unit vector.
  /// Since the measures of this axis in either frame F or M are the same (see
  /// this class's documentation for frames's definitions) then,
  /// `axis = axis_F = axis_M`.
  const Vector3<double>& translation_axis() const {
    return axis_;
  }

  /// @name Context-dependent value access
  ///
  /// These methods require the provided context to be an instance of
  /// MultibodyTreeContext. Failure to do so leads to a std::logic_error.
  /// @{

  /// Gets the translation distance of `this` mobilizer from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The translation coordinate of `this` joint read from `context`.
  const T& get_translation(const Context<T>& context) const {
    return get_mobilizer()->get_translation(context);
  }

  /// Sets `context` so that the generalized coordinate corresponding to the
  /// translation distance of `this` joint equals `translation`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @param[in] translation
  ///   The desired translation in meters to be stored in `context`.
  /// @returns a constant reference to `this` joint.
  const PrismaticJoint<T>& set_translation(
      Context<T>* context, const T& translation) const {
    get_mobilizer()->set_translation(context, translation);
    return *this;
  }

  /// Gets the rate of change, in meters per second, of `this` joint's
  /// translation distance (see get_translation()) from `context`.
  /// @param[in] context
  ///   The context of the MultibodyTree this joint belongs to.
  /// @returns The rate of change of `this` joint's translation read from
  /// `context`.
  const T& get_translation_rate(const Context<T>& context) const {
    return get_mobilizer()->get_translation_rate(context);
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
    get_mobilizer()->set_translation_rate(context, translation_dot);
    return *this;
  }

  /// @}

  /// Adds into `multibody_forces` a given `force`, in Newtons, for `this` joint
  /// that is to be applied along the joint's axis. The force is defined to be
  /// positive in the direction along this joint's axis.
  /// That is, a positive force causes a positive translational acceleration
  /// along the joint's axis.
  void AddInForce(
      const systems::Context<T>& context,
      const T& force,
      MultibodyForces<T>* multibody_forces) const {
    DRAKE_DEMAND(multibody_forces != nullptr);
    DRAKE_DEMAND(
        multibody_forces->CheckHasRightSizeForModel(this->get_parent_tree()));
    this->AddInOneForce(context, 0, force, multibody_forces);
  }

 protected:
  /// Joint<T> virtual override called through public NVI, Joint::AddInForce().
  /// Therefore arguments were already checked to be valid.
  /// For a %PrismaticJoint, we must always have `joint_dof = 0` since there is
  /// only a single degree of freedom (get_num_dofs() == 1). `joint_tau` is the
  /// linear force applied along the joint's axis, on the body declared as child
  /// (according to the prismatic joint's constructor) at the origin of the
  /// child frame (which is coincident with the origin of the parent frame at
  /// all times).
  void DoAddInOneForce(
      const systems::Context<T>&,
      int joint_dof,
      const T& joint_tau,
      MultibodyForces<T>* forces) const final {
    // Right now we assume all the forces in joint_tau go into a single
    // mobilizer.
    Eigen::VectorBlock<Eigen::Ref<VectorX<T>>> tau_mob =
        get_mobilizer()->get_mutable_generalized_forces_from_array(
            &forces->mutable_generalized_forces());
    tau_mob(joint_dof) += joint_tau;
  }

 private:
  int do_get_num_dofs() const final {
    return 1;
  }

  // Joint<T> finals:
  std::unique_ptr<typename Joint<T>::BluePrint>
  MakeImplementationBlueprint() const final {
    auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
    blue_print->mobilizers_.push_back(
        std::make_unique<PrismaticMobilizer<T>>(
            this->frame_on_parent(), this->frame_on_child(), axis_));
    return std::move(blue_print);
  }

  std::unique_ptr<Joint<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  std::unique_ptr<Joint<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  // Make PrismaticJoint templated on every other scalar type a friend of
  // PrismaticJoint<T> so that CloneToScalar<ToAnyOtherScalar>() can access
  // private members of PrismaticJoint<T>.
  template <typename> friend class PrismaticJoint;

  // Friend class to facilitate testing.
  friend class JointTester;

  // Returns the mobilizer implementing this joint.
  // The internal implementation of this joint could change in a future version.
  // However its public API should remain intact.
  const PrismaticMobilizer<T>* get_mobilizer() const {
    // This implementation should only have one mobilizer.
    DRAKE_DEMAND(this->get_implementation().num_mobilizers() == 1);
    const PrismaticMobilizer<T>* mobilizer =
        dynamic_cast<const PrismaticMobilizer<T>*>(
            this->get_implementation().mobilizers_[0]);
    DRAKE_DEMAND(mobilizer != nullptr);
    return mobilizer;
  }

  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Joint<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // This is the joint's axis expressed in either M or F since axis_M = axis_F.
  // It is a unit vector.
  Vector3<double> axis_;
};

}  // namespace multibody
}  // namespace drake
