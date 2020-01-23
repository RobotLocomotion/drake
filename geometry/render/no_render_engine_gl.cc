
#include "drake/geometry/render/render_engine_gl.h"

namespace drake {
namespace geometry {
namespace render {

using math::RigidTransformd;
using systems::sensors::ImageDepth32F;
using systems::sensors::ImageLabel16I;
using systems::sensors::ImageRgba8U;

void RenderEngineGl::UpdateViewpoint(const RigidTransformd& X_WR) {
}

void RenderEngineGl::RenderColorImage(const CameraProperties&, bool,
                                      ImageRgba8U*) const {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

void RenderEngineGl::RenderDepthImage(const DepthCameraProperties& camera,
                                      ImageDepth32F* depth_image_out) const {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

void RenderEngineGl::RenderLabelImage(const CameraProperties&, bool,
                                      ImageLabel16I*) const {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

bool RenderEngineGl::DoRegisterVisual(GeometryId id, const Shape& shape,
                                      const PerceptionProperties&,
                                      const RigidTransformd& X_FG) {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

void RenderEngineGl::DoUpdateVisualPose(GeometryId id,
                                        const RigidTransformd& X_WG) {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

bool RenderEngineGl::DoRemoveGeometry(GeometryId id) {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

std::unique_ptr<RenderEngine> RenderEngineGl::DoClone() const {
  throw std::runtime_error(
      "RenderEngineGl was not compiled. You'll need to use a different render "
      "engine.");
}

}  // namespace render
}  // namespace geometry
}  // namespace drake