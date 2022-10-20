#include "drake/multibody/tree/prismatic_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree-inl.h"

namespace drake {
namespace multibody {

template <typename T>
PrismaticJoint<T>::PrismaticJoint(
    const std::string& name, const Frame<T>& frame_on_parent,
    const Frame<T>& frame_on_child, const Vector3<double>& axis,
    double pos_lower_limit, double pos_upper_limit, double damping)
    : Joint<T>(name, frame_on_parent, frame_on_child,
               VectorX<double>::Constant(1, damping),
               VectorX<double>::Constant(1, pos_lower_limit),
               VectorX<double>::Constant(1, pos_upper_limit),
               VectorX<double>::Constant(
                   1, -std::numeric_limits<double>::infinity()),
               VectorX<double>::Constant(
                   1, std::numeric_limits<double>::infinity()),
               VectorX<double>::Constant(
                   1, -std::numeric_limits<double>::infinity()),
               VectorX<double>::Constant(
                   1, std::numeric_limits<double>::infinity())) {
  const double kEpsilon = std::sqrt(std::numeric_limits<double>::epsilon());
  if (axis.isZero(kEpsilon)) {
    throw std::logic_error("Prismatic joint axis vector must have nonzero "
                           "length.");
  }
  if (damping < 0) {
    throw std::logic_error("Prismatic joint damping must be nonnegative.");
  }
  axis_ = axis.normalized();
}

template <typename T>
const std::string& PrismaticJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> PrismaticJoint<T>::TemplatedDoCloneToScalar(
      const Frame<ToScalar>& frame_on_parent_body_clone,
      const Frame<ToScalar>& frame_on_child_body_clone) const {

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<PrismaticJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->translation_axis(), this->position_lower_limit(),
      this->position_upper_limit(), this->damping());
  joint_clone->set_velocity_limits(this->velocity_lower_limits(),
                                   this->velocity_upper_limits());
  joint_clone->set_acceleration_limits(this->acceleration_lower_limits(),
                                       this->acceleration_upper_limits());
  joint_clone->set_default_positions(this->default_positions());

  return joint_clone;
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>>
PrismaticJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  return TemplatedDoCloneToScalar(frame_on_parent_body_clone,
                                  frame_on_child_body_clone);
}

template <typename T>
std::unique_ptr<Joint<double>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> PrismaticJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
const Joint<T>& PrismaticJoint<T>::DoCloneTo(
    internal::MultibodyTree<T>* tree, const Frame<T>& dest_frame_on_parent,
    const Frame<T>& dest_frame_on_child) const {
  return tree->AddJoint(TemplatedDoCloneToScalar(dest_frame_on_parent,
                                                 dest_frame_on_child));
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::PrismaticJoint)
