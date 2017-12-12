#include "drake/systems/sensors/rgbd_renderer.h"

namespace drake {
namespace systems {
namespace sensors {

const int kNumMaxLabel = 256;

RgbdRenderer::RgbdRenderer(const Eigen::Isometry3d& X_WR, int width, int height,
               double z_near, double z_far, double fov_y, bool show_window)
      : width_(width),
        height_(height),
        fov_y_(fov_y),
        z_near_(z_near),
        z_far_(z_far),
        show_window_(show_window),
        color_palette_(kNumMaxLabel, Label::kFlatTerrain, Label::kNoBody) {}

RgbdRenderer::~RgbdRenderer() {}

optional<RgbdRenderer::VisualIndex> RgbdRenderer::RegisterVisual(
    const DrakeShapes::VisualElement& visual, int body_id) {
  return DoRegisterVisual(visual, body_id);
}

void RgbdRenderer::AddFlatTerrain() {
  DoAddFlatTerrain();
}

void RgbdRenderer::UpdateViewpoint(const Eigen::Isometry3d& X_WR) const {
  DoUpdateViewpoint(X_WR);
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

int RgbdRenderer::width() const { return width_; }

int RgbdRenderer::height() const { return height_; }

double RgbdRenderer::fov_y() const { return fov_y_; }

const ColorI& RgbdRenderer::get_sky_color() const {
  return color_palette_.get_sky_color();
}

const ColorI& RgbdRenderer::get_flat_terrain_color() const {
  return color_palette_.get_terrain_color();
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
