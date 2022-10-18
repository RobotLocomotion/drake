#include "drake/multibody/tree/revolute_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
RevoluteJoint<T>::RevoluteJoint(const std::string& name,
              const Frame<T>& frame_on_parent, const Frame<T>& frame_on_child,
              const Vector3<double>& axis, double pos_lower_limit,
              double pos_upper_limit, double damping)
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
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  if (axis.isZero(kEpsilon)) {
    throw std::logic_error("Revolute joint axis vector must have nonzero "
                           "length.");
  }
  if (damping < 0) {
    throw std::logic_error("Revolute joint damping must be nonnegative.");
  }
  axis_ = axis.normalized();
}

template <typename T>
const std::string& RevoluteJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> RevoluteJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<RevoluteJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->revolute_axis(), this->position_lower_limits()[0],
      this->position_upper_limit(), this->damping());
  joint_clone->set_velocity_limits(this->velocity_lower_limits(),
                                   this->velocity_upper_limits());
  joint_clone->set_acceleration_limits(this->acceleration_lower_limits(),
                                       this->acceleration_upper_limits());
  joint_clone->set_default_positions(this->default_positions());

  return joint_clone;
}

template <typename T>
std::unique_ptr<Joint<double>> RevoluteJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> RevoluteJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> RevoluteJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<typename Joint<T>::BluePrint>
RevoluteJoint<T>::MakeImplementationBlueprint() const {
  auto blue_print = std::make_unique<typename Joint<T>::BluePrint>();
  auto revolute_mobilizer = std::make_unique<internal::RevoluteMobilizer<T>>(
      this->frame_on_parent(), this->frame_on_child(), axis_);
  revolute_mobilizer->set_default_position(this->default_positions());
  blue_print->mobilizers_.push_back(std::move(revolute_mobilizer));
  return blue_print;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::RevoluteJoint)
