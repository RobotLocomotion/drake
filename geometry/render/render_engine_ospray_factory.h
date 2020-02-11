#pragma once

#include <memory>
#include <optional>

#include "drake/geometry/render/render_engine.h"

namespace drake {
namespace geometry {
namespace render {

/** The mode in which the RenderEngineOspray performs.

 The ray tracer can produce hard shadows and doesn't depend on the samples per
 pixel value. Ray tracing produces *simple* illumination effects at a relatively
 low computation cost.

 The path tracer produces complex global illumination and depends on the samples
 per pixel. Path tracing produces *complex* illumination effects (caustics,
 indirect illumination, soft shadows, etc.) at a high computation cost.  */
enum class OsprayMode {
  kRayTracer,
  kPathTracer
};

// TODO(SeanCurtis-TRI): Add configuration features:
//  - require ospray-specific materials -- i.e., don't accept (phong, diffuse)
//    as a material definition.
//  - background texture
/** Construction parameters for the RenderEngineOspray.  */
struct RenderEngineOsprayParams {
  /** The rendering mode to use.  */
  OsprayMode mode{OsprayMode::kPathTracer};

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineOspray.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  // TODO(SeanCurtis-TRI): Reconcile this with a specified background image.
  /** The rgb color for the environment background (each channel in the range
   [0, 1]). The default value is the same color as in the equivalent byte-valued
   rgb triple [204, 229, 255].  */
  std::optional<Eigen::Vector3d> background_color{};

  /** The number of illumination samples per pixel. Higher numbers introduce
   higher quality at increased cost. Only has an effect if mode is
   OsprayMode::kPathTracer.  */
  int samples_per_pixel{1};

  /** Whether to turn on shadows when in the `OsprayMode::kRayTracer` rendering
   mode. It is ignored in other modes.  */
  bool use_shadows{true};
};

/** Constructs a RenderEngine implementation which uses an OSPRay-based
 renderer.

 @anchor render_engine_ospray_properties
 <h2>Geometry perception properties</h2>

 This RenderEngine implementation looks for the following properties when
 registering visual geometry.

 <h3>RGB images</h3>

 | Group name | Property Name | Required |  Property Type  | Property Description |
 | :--------: | :-----------: | :------: | :-------------: | :------------------- |
 |    phong   | diffuse       | no¹      | Eigen::Vector4d | The rgba value of the object surface. |

 ¹ If no diffuse value is given, a default rgba value will be applied. The
   default color is a bright orange. This default value can be changed to a
   different value at construction.

 @warning This RenderEngine implementation contains a sophisticated renderer
 for advanced lighting and material affects. The above documented behavior is
 the smallest slice. In the future, the advanced features will be exposed and
 it will change the properties that expect to be provided.

 <h3>Depth images</h3>

 This RenderEngine implementation does not currently support depth images.

 <h3>Label images</h3>

 This RenderEngine implementation does not currently support label images.

 <h3>Geometries accepted by %RenderEngineVtk</h3>

 As documented in RenderEngine::RegisterVisual(), a RenderEngine implementation
 can use the properties found in the PerceptionProperties to determine whether
 it _accepts_ a shape provided for registration. this RenderEngine
 implementation makes use of defaults to accept _all_ geometries.

 @warning When the API extends to include advanced materials, this will change
 to only accept geometries that have valid OSPRay materials defined.  */
std::unique_ptr<RenderEngine> MakeRenderEngineOspray(
    const RenderEngineOsprayParams& params);

}  // namespace render
}  // namespace geometry
}  // namespace drake
