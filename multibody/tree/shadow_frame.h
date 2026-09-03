#pragma once

#include <memory>
#include <optional>
#include <string>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/tree/frame.h"

namespace drake {
namespace multibody {
namespace internal {

/* A Frame fixed to an ephemeral shadow link, coincident at all times with a
"source" Frame fixed to that shadow's primary link.

When we model a closed kinematic loop we break the loop by splitting a link
into a primary link and one or more ephemeral shadow links, retargeting one of
the loop joint's frames onto a shadow (see LinkJointGraph). The joint's frame
was authored on the primary link, so the mobilizer needs an equivalent frame on
the shadow link. Because a shadow's link frame is intended to coincide with its
primary's (when the weld constraint is satisfied), "equivalent" here just means
having the same local pose: only the link the frame is fixed to differs, not
its pose on the link to which it is fixed.

%ShadowFrame delegates its pose to the source frame rather than storing a pose
of its own, so it declares no Context parameters. That makes the source frame
the single source of truth: a runtime change to the source frame's pose (say
FixedOffsetFrame::SetPoseInParentFrame()) is reflected on the shadow side
automatically. The two frames are therefore coincident when the weld
constraint is satisfied.

@tparam_default_scalar */
template <typename T>
class ShadowFrame final : public Frame<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ShadowFrame);

  /* Creates a frame named `name`, fixed to `shadow_link` and coincident with
  `source_frame`. If `model_instance` is unspecified, uses `shadow_link`'s.
  @pre `source_frame` is fixed to `shadow_link`'s primary link. */
  ShadowFrame(const std::string& name, const Link<T>& shadow_link,
              const Frame<T>& source_frame,
              std::optional<ModelInstanceIndex> model_instance = {});

  ~ShadowFrame() final;

  /* Returns the frame on the primary link that this frame mirrors. */
  const Frame<T>& source_frame() const { return source_frame_; }

  math::RigidTransform<T> GetFixedPoseInBodyFrame() const final {
    // The shadow link frame coincides with the primary's, so the source
    // frame's pose in its own link frame is also this frame's pose in ours.
    return source_frame_.GetFixedPoseInBodyFrame();
  }

  math::RotationMatrix<T> GetFixedRotationMatrixInBodyFrame() const final {
    return source_frame_.GetFixedRotationMatrixInBodyFrame();
  }

 protected:
  /* @pre The source frame already has a clone in `tree_clone`. */
  std::unique_ptr<Frame<double>> DoCloneToScalar(
      const MultibodyTree<double>& tree_clone) const final;

  /* @pre The source frame already has a clone in `tree_clone`. */
  std::unique_ptr<Frame<AutoDiffXd>> DoCloneToScalar(
      const MultibodyTree<AutoDiffXd>& tree_clone) const final;

  std::unique_ptr<Frame<symbolic::Expression>> DoCloneToScalar(
      const MultibodyTree<symbolic::Expression>& tree_clone) const final;

  std::unique_ptr<Frame<T>> DoShallowClone() const final;

  math::RigidTransform<T> DoCalcPoseInBodyFrame(
      const systems::Parameters<T>& parameters) const final;

  math::RotationMatrix<T> DoCalcRotationMatrixInBodyFrame(
      const systems::Parameters<T>& parameters) const final;

 private:
  // Helper method to make a clone templated on ToScalar.
  template <typename ToScalar>
  std::unique_ptr<Frame<ToScalar>> TemplatedDoCloneToScalar(
      const MultibodyTree<ToScalar>& tree_clone) const;

  // The frame on the primary link whose pose this frame mirrors.
  const Frame<T>& source_frame_;
};

}  // namespace internal
}  // namespace multibody
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::multibody::internal::ShadowFrame);
