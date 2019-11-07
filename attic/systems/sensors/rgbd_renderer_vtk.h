#pragma once

#include <memory>

#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

/// An RgbdRenderer implementation using VTK.
class RgbdRendererVTK final : public RgbdRenderer {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(RgbdRendererVTK)

  RgbdRendererVTK(
      const RenderingConfig& config,
      const Eigen::Isometry3d& X_WC = Eigen::Isometry3d::Identity());

  ~RgbdRendererVTK();

 private:
  void ImplAddFlatTerrain() override;

  std::optional<VisualIndex> ImplRegisterVisual(
      const DrakeShapes::VisualElement& visual, int body_id) override;

  void ImplUpdateVisualPose(const Eigen::Isometry3d& X_WV,
                        int body_id, VisualIndex visual_id) const override;

  void ImplUpdateViewpoint(const Eigen::Isometry3d& X_WC) const override;

  void ImplRenderColorImage(ImageRgba8U* color_image_out) const override;

  void ImplRenderDepthImage(ImageDepth32F* depth_image_out) const override;

  void ImplRenderLabelImage(ImageLabel16I* label_image_out) const override;

  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
