#pragma once

#include <memory>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** Temporary dummy implementation of a purely OpenGL-based render engine. Due
 to the lack of support on Mac, this has been initially introduced to verify
 platform specific (i.e. Ubuntu only) conditionals for building and installing.
 At minimum, RenderDepthImage has been implemented to return a static image.
 The rest of the implementation will be ported over once the conditionalizing
 is no longer considered a risk.
 TODO(tehbelinda): Complete port of RenderEngineGl.  */
class RenderEngineGl final : public RenderEngine {
 public:
  /** @see RenderEngine::UpdateViewpoint().  */
  void UpdateViewpoint(const math::RigidTransformd& X_WR) final;

  /** @see RenderEngine::RenderColorImage().  */
  void RenderColorImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageRgba8U* color_image_out) const final;

  /** @see RenderEngine::RenderDepthImage().  */
  void RenderDepthImage(
      const DepthCameraProperties& camera,
      systems::sensors::ImageDepth32F* depth_image_out) const final;

  /** @see RenderEngine::RenderLabelImage().  */
  void RenderLabelImage(
      const CameraProperties& camera, bool show_window,
      systems::sensors::ImageLabel16I* label_image_out) const final;

 private:
  // @see RenderEngine::DoRegisterVisual().
  bool DoRegisterVisual(GeometryId id, const Shape& shape,
                        const PerceptionProperties& properties,
                        const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoUpdateVisualPose().
  void DoUpdateVisualPose(GeometryId id,
                          const math::RigidTransformd& X_WG) final;

  // @see RenderEngine::DoRemoveGeometry().
  bool DoRemoveGeometry(GeometryId id) final;

  // @see RenderEngine::DoClone().
  std::unique_ptr<RenderEngine> DoClone() const final;
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
