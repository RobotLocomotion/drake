#include "drake/geometry/render/gl_renderer/render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

RenderEngineGl::RenderEngineGl() {
}

void RenderEngineGl::UpdateViewpoint(const RigidTransformd&) {}

void RenderEngineGl::RenderColorImage(const CameraProperties&, bool,
                                      ImageRgba8U*) const {
}

void RenderEngineGl::RenderDepthImage(const DepthCameraProperties&,
                                      ImageDepth32F*) const {
}

void RenderEngineGl::RenderLabelImage(const CameraProperties&, bool,
                                      ImageLabel16I*) const {
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
