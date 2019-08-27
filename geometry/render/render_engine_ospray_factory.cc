#include "drake/geometry/render/render_engine_ospray_factory.h"

#include "drake/geometry/render/render_engine_ospray.h"

namespace drake {
namespace geometry {
namespace render {

std::unique_ptr<RenderEngine> MakeRenderEngineOspray(
    const RenderEngineOsprayParams& params) {
  return std::make_unique<RenderEngineOspray>(params);
}

}  // namespace render
}  // namespace geometry
}  // namespace drake
