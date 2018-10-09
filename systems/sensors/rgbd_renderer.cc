#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

const int kNumMaxLabel = 256;

RgbdRenderer::RgbdRenderer(const RenderingConfig& config,
                           const Eigen::Isometry3d&)
    : config_(config),
      color_palette_(kNumMaxLabel, Label::kFlatTerrain, Label::kNoBody) {
}

RgbdRenderer::~RgbdRenderer() {}

optional<RgbdRenderer::VisualIndex> RgbdRenderer::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return ImplRegisterVisual(visual, body_id);
}

void RgbdRenderer::AddFlatTerrain() {
  ImplAddFlatTerrain();
}

void RgbdRenderer::UpdateViewpoint(const Eigen::Isometry3d& X_WC) const {
  ImplUpdateViewpoint(X_WC);
}

void RgbdRenderer::UpdateVisualPose(
    const Eigen::Isometry3d& X_WV, int body_id, VisualIndex visual_id) const {
  ImplUpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRenderer::RenderColorImage(ImageRgba8U* color_image_out) const {
  ImplRenderColorImage(color_image_out);
}

void RgbdRenderer::RenderDepthImage(ImageDepth32F* depth_image_out) const {
  ImplRenderDepthImage(depth_image_out);
}

void RgbdRenderer::RenderLabelImage(ImageLabel16I* label_image_out) const {
  ImplRenderLabelImage(label_image_out);
}

const RenderingConfig& RgbdRenderer::config() const { return config_; }

const ColorPalette<int>& RgbdRenderer::color_palette() const {
  return color_palette_;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
