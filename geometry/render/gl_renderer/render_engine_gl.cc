
#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

RenderEngineGl::RenderEngineGl() {
  gl::OpenGlContext::Dummy();
}

void RenderEngineGl::UpdateViewpoint(const RigidTransformd&) {}

void RenderEngineGl::RenderColorImage(const CameraProperties&, bool,
                                      ImageRgba8U*) const {
  throw std::runtime_error("RenderEngineGl cannot render color images");
}

void RenderEngineGl::RenderDepthImage(const DepthCameraProperties& camera,
                                      ImageDepth32F* depth_image_out) const {
  // Make up a dummy image for now.
  for (int v = 0; v < camera.height; ++v) {
    for (int u = 0; u < camera.width; ++u) {
      depth_image_out->at(u, v)[0] = 0.5;
    }
  }
}

void RenderEngineGl::RenderLabelImage(const CameraProperties&, bool,
                                      ImageLabel16I*) const {
  throw std::runtime_error("RenderEngineGl cannot render label images");
}

bool RenderEngineGl::DoRegisterVisual(GeometryId, const Shape&,
                                      const PerceptionProperties&,
                                      const RigidTransformd&) {
  return true;
}

void RenderEngineGl::DoUpdateVisualPose(GeometryId,
                                        const RigidTransformd&) {
}

bool RenderEngineGl::DoRemoveGeometry(GeometryId) {
  return false;
}

std::unique_ptr<RenderEngine> RenderEngineGl::DoClone() const {
  return std::unique_ptr<RenderEngineGl>(new RenderEngineGl(*this));
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
