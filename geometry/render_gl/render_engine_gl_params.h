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
  }

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};

  /** Lights in the scene. More than five lights is an error. If no lights are
   defined, a single directional light, fixed to the camera frame, is used. */
  std::vector<render::LightParameter> lights;
};

}  // namespace geometry
}  // namespace drake
