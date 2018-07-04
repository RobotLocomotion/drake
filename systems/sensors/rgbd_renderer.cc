#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

const int kNumMaxLabel = 256;

RgbdRendererBase::RgbdRenderer(const RenderingConfig& config,
                           const Eigen::Isometry3d&)
    : config_(config),
      color_palette_(kNumMaxLabel, Label::kFlatTerrain, Label::kNoBody) {
}

RgbdRendererBase::~RgbdRenderer() {}

optional<RgbdRendererBase::VisualIndex> RgbdRenderer::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return ImplRegisterVisual(visual, body_id);
}

void RgbdRendererBase::AddFlatTerrain() {
  ImplAddFlatTerrain();
}

void RgbdRendererBase::UpdateViewpoint(const Eigen::Isometry3d& X_WC) const {
  ImplUpdateViewpoint(X_WC);
}

void RgbdRendererBase::UpdateVisualPose(
    const Eigen::Isometry3d& X_WV, int body_id, VisualIndex visual_id) const {
  ImplUpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRendererBase::RenderColorImage(ImageRgba8U* color_image_out) const {
  ImplRenderColorImage(color_image_out);
}

void RgbdRendererBase::RenderDepthImage(ImageDepth32F* depth_image_out) const {
  ImplRenderDepthImage(depth_image_out);
}

void RgbdRendererBase::RenderLabelImage(ImageLabel16I* label_image_out) const {
  ImplRenderLabelImage(label_image_out);
}

const RenderingConfig& RgbdRendererBase::config() const { return config_; }

const ColorPalette& RgbdRendererBase::color_palette() const {
  return color_palette_;
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
