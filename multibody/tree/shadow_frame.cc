#include "drake/multibody/tree/shadow_frame.h"

#include <memory>
#include <optional>

#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/rigid_body.h"

namespace drake {
namespace multibody {
namespace internal {

template <typename T>
ShadowFrame<T>::ShadowFrame(const std::string& name, const Link<T>& shadow_link,
                            const Frame<T>& source_frame,
                            std::optional<ModelInstanceIndex> model_instance)
    : Frame<T>(name, shadow_link, model_instance),
      source_frame_(source_frame) {}

template <typename T>
ShadowFrame<T>::~ShadowFrame() = default;

template <typename T>
template <typename ToScalar>
std::unique_ptr<Frame<ToScalar>> ShadowFrame<T>::TemplatedDoCloneToScalar(
    const MultibodyTree<ToScalar>& tree_clone) const {
  const Link<ToScalar>& shadow_link_clone =
      tree_clone.get_variant(this->link());
  const Frame<ToScalar>& source_frame_clone =
      tree_clone.get_variant(source_frame_);
  auto new_frame = std::make_unique<ShadowFrame<ToScalar>>(
      this->name(), shadow_link_clone, source_frame_clone);
  new_frame->set_is_ephemeral(this->is_ephemeral());
  return new_frame;
}

template <typename T>
std::unique_ptr<Frame<double>> ShadowFrame<T>::DoCloneToScalar(
    const MultibodyTree<double>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<AutoDiffXd>> ShadowFrame<T>::DoCloneToScalar(
    const MultibodyTree<AutoDiffXd>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<symbolic::Expression>> ShadowFrame<T>::DoCloneToScalar(
    const MultibodyTree<symbolic::Expression>& tree_clone) const {
  return TemplatedDoCloneToScalar(tree_clone);
}

template <typename T>
std::unique_ptr<Frame<T>> ShadowFrame<T>::DoShallowClone() const {
  auto new_frame = std::make_unique<ShadowFrame<T>>(this->name(), this->link(),
                                                    source_frame_);
  new_frame->set_is_ephemeral(this->is_ephemeral());
  return new_frame;
}

template <typename T>
math::RigidTransform<T> ShadowFrame<T>::DoCalcPoseInBodyFrame(
    const systems::Parameters<T>& parameters) const {
  // Our link frame coincides with the source frame's link frame, so our X_LF
  // is exactly the source frame's X_LF. Delegating (rather than storing a
  // parameter of our own) is what keeps the two frames in agreement when the
  // source frame's pose is changed at runtime.
  return source_frame_.CalcOffsetPoseInBody(
      parameters, math::RigidTransform<T>::Identity());
}

template <typename T>
math::RotationMatrix<T> ShadowFrame<T>::DoCalcRotationMatrixInBodyFrame(
    const systems::Parameters<T>& parameters) const {
  return source_frame_.CalcOffsetRotationMatrixInBody(
      parameters, math::RotationMatrix<T>::Identity());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::ShadowFrame);
