#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

const int kNumMaxLabel = 256;

RgbdRenderer::RgbdRenderer(const RenderingConfig& config,
                           const Eigen::Isometry3d& X_WC)
    : config_(config),
      color_palette_(kNumMaxLabel, Label::kFlatTerrain, Label::kNoBody) {
  (void)X_WC;
}

RgbdRenderer::~RgbdRenderer() {}

optional<RgbdRenderer::VisualIndex> RgbdRenderer::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return DoRegisterVisual(visual, body_id);
}

void RgbdRenderer::AddFlatTerrain() {
  DoAddFlatTerrain();
}

void RgbdRenderer::UpdateViewpoint(const Eigen::Isometry3d& X_WC) const {
  DoUpdateViewpoint(X_WC);
}

void RgbdRenderer::UpdateVisualPose(
    const Eigen::Isometry3d& X_WV, int body_id, VisualIndex visual_id) const {
  DoUpdateVisualPose(X_WV, body_id, visual_id);
}

void RgbdRenderer::RenderColorImage(ImageRgba8U* color_image_out) const {
  DoRenderColorImage(color_image_out);
}

void RgbdRenderer::RenderDepthImage(ImageDepth32F* depth_image_out) const {
  DoRenderDepthImage(depth_image_out);
}

void RgbdRenderer::RenderLabelImage(ImageLabel16I* label_image_out) const {
  DoRenderLabelImage(label_image_out);
}

const RenderingConfig& RgbdRenderer::config() const { return config_; }

const ColorPalette& RgbdRenderer::color_palette() const {
  return color_palette_;
}

const ColorI& RgbdRenderer::get_sky_color() const {
  return color_palette_.get_sky_color();
}

const ColorI& RgbdRenderer::get_flat_terrain_color() const {
  return color_palette_.get_terrain_color();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
