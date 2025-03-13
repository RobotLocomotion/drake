#include "drake/multibody/tree/screw_joint.h"

#include <memory>
#include <stdexcept>

#include "drake/multibody/tree/multibody_tree.h"

namespace drake {
namespace multibody {

template <typename T>
ScrewJoint<T>::ScrewJoint(const std::string& name,
                          const Frame<T>& frame_on_parent,
                          const Frame<T>& frame_on_child,
                          const Vector3<double>& axis, double screw_pitch,
                          double damping)
    : Joint<T>(
          name, frame_on_parent, frame_on_child,
          VectorX<double>::Constant(1, damping),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1, std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1, std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    -std::numeric_limits<double>::infinity()),
          VectorX<double>::Constant(1,
                                    std::numeric_limits<double>::infinity())),
      screw_pitch_{screw_pitch} {
  const double kEpsilon = std::numeric_limits<double>::epsilon();
  if (axis.isZero(kEpsilon)) {
    throw std::logic_error("Screw joint axis vector must have nonzero length.");
  }
  if (damping < 0) {
    throw std::logic_error("Screw joint damping must be nonnegative.");
  }
  axis_ = axis.normalized();
}

template <typename T>
ScrewJoint<T>::~ScrewJoint() = default;

template <typename T>
const std::string& ScrewJoint<T>::type_name() const {
  static const never_destroyed<std::string> name{kTypeName};
  return name.access();
}

template <typename T>
template <typename ToScalar>
std::unique_ptr<Joint<ToScalar>> ScrewJoint<T>::TemplatedDoCloneToScalar(
    const internal::MultibodyTree<ToScalar>& tree_clone) const {
  const Frame<ToScalar>& frame_on_parent_body_clone =
      tree_clone.get_variant(this->frame_on_parent());
  const Frame<ToScalar>& frame_on_child_body_clone =
      tree_clone.get_variant(this->frame_on_child());

  // Make the Joint<T> clone.
  auto joint_clone = std::make_unique<ScrewJoint<ToScalar>>(
      this->name(), frame_on_parent_body_clone, frame_on_child_body_clone,
      this->screw_axis(), this->screw_pitch(), this->default_damping());
  joint_clone->set_position_limits(this->position_lower_limits(),
                                   this->position_upper_limits());
  joint_clone->set_velocity_limits(this->velocity_lower_limits(),
                                   this->velocity_upper_limits());
  joint_clone->set_acceleration_limits(this->acceleration_lower_limits(),
                                       this->acceleration_upper_limits());
  joint_clone->set_default_positions(this->default_positions());

  return joint_clone;
}

template <typename T>
std::unique_ptr<Joint<double>> ScrewJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<AutoDiffXd>> ScrewJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<symbolic::Expression>> ScrewJoint<T>::DoCloneToScalar(
    const internal::MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Joint<T>> ScrewJoint<T>::DoShallowClone() const {
  return std::make_unique<ScrewJoint<T>>(
      this->name(), this->frame_on_parent(), this->frame_on_child(),
      this->screw_axis(), this->screw_pitch(), this->default_damping());
}

// N.B. Due to esoteric linking errors on Mac (see #9345) involving
// `MobilizerImpl`, we must place this implementation in the source file, not
// in the header file.
template <typename T>
std::unique_ptr<internal::Mobilizer<T>> ScrewJoint<T>::MakeMobilizerForJoint(
    const internal::SpanningForest::Mobod& mobod,
    internal::MultibodyTree<T>*) const {
  const auto [inboard_frame, outboard_frame] =
      this->tree_frames(mobod.is_reversed());
  // TODO(sherm1) The mobilizer needs to be reversed, not just the frames.
  auto screw_mobilizer = std::make_unique<internal::ScrewMobilizer<T>>(
      mobod, *inboard_frame, *outboard_frame, this->screw_axis(), screw_pitch_);
  screw_mobilizer->set_default_position(this->default_positions());
  return screw_mobilizer;
}

}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class ::drake::multibody::ScrewJoint);
