#pragma once

#include <memory>

#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

/// An RgbdRenderer implementation using VTK
class RgbdRendererVTK final : public RgbdRenderer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdRendererVTK)

  RgbdRendererVTK(
      const RenderingConfig& config,
      const Eigen::Isometry3d& X_WC = Eigen::Isometry3d::Identity());

  ~RgbdRendererVTK();

 private:
  void DoAddFlatTerrain() override;

  optional<VisualIndex> DoRegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id) override;

  void DoUpdateVisualPose(const Eigen::Isometry3d& X_WV,
                        int body_id, VisualIndex visual_id) const override;

  void DoUpdateViewpoint(const Eigen::Isometry3d& X_WC) const override;

  void DoRenderColorImage(ImageRgba8U* color_image_out) const override;

  void DoRenderDepthImage(ImageDepth32F* depth_image_out) const override;

  void DoRenderLabelImage(ImageLabel16I* label_image_out) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
