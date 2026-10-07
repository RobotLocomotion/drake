#pragma once

#include <vector>

#include "drake/common/name_value.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_label.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** Construction parameters for RenderEngineGl.  */
struct RenderEngineGlParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
    a->Visit(DRAKE_NVP(lights));
    a->Visit(DRAKE_NVP(cast_shadows));
    a->Visit(DRAKE_NVP(shadow_map_size));
  }

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};

  /** Lights in the scene. If no lights are defined, a single directional
   light, fixed to the camera frame, is used.

   Note: RenderEngineGl does not have a hard-coded limit on the number of
         lights, but more lights increases rendering cost. */
  std::vector<render::LightParameter> lights;

  /** If `true`, *all* lights that are *able* to cast shadows will do so.

  Several important notes when designing your lighting:

  - Point lights do not cast shadows.
  - Spot lights will not cast shadows if the spot light angle is 90 degrees
    or more. At 90 degrees, the spot light is now a half-point light.
    Even though 89.9 degrees _would_ enable shadows, it is still an
    impractical value. To actually see shadows, the `shadow_map_size` value
    would have to be absurdly large to support such a wide extent.
  - Directional lights will create a shadow map that spans the whole scene.
    If your scene includes a geometry that is significantly larger than
    the locale you're rendering, this will significantly reduce the
    efficacy of the directional light's shadows. Consider truncating that
    larger geometry. A common case would be to use a HalfSpace to define
    a ground. A half space has infinite extent, so any reasonable
    approximation would be quite large. Better to use a box targeted to
    where you need it.
  - Shadows may have unexpected behavior with transparent objects.
    - Transparent objects will receive shadows.
    - If the object is rendered transparent in the diffuse color's alpha
      channel (i.e., any alpha value less than 1.0), the object will not
      cast shadows.
    - If the object has full diffuse color alpha, but it has a diffuse map
      with non opaque texels, the object *will* cast shadows and the
      shadows will completely ignore the texel transparency (the whole mesh
      will be treated as fully opaque).
  - If you only plan to render depth or label images, leave this as false.
    Setting it as true will increase start up time and GPU memory usage to
    support shadows that will never be used.

  Currently, there is no way to enable/disable shadows on a per-light basis.

  <!-- TODO(SeanCurtis-TRI): We want to enable per-light shadow configuration.
   We need to do it in lock step with RenderEngineVtk. --> */
  bool cast_shadows{false};

  /** The size of texture map (in pixels) to use for shadow maps. Note: this is
   a *global* setting. All shadow casting lights will use a map of the same
   size. Larger map sizes increase GPU memory usage and rendering times but
   improve shadow fidelity (less obvious pixelation).

   See the note on `cast_shadows` for the warning on directional lights and
   shadow maps.

   @pre shadow_map_size is positive and does not exceed the OpenGL
   implementation's maximum allowable texture size. Powers of two are best. */
  int shadow_map_size{256};
};

}  // namespace geometry
}  // namespace drake
