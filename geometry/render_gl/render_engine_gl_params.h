#pragma once

#include <string>

#include "drake/common/name_value.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {

/** Construction parameters for RenderEngineGl.  */
struct RenderEngineGlParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_label));
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
  }

  /** Default render label policy to apply to a geometry when no render label
   has been assigned. Should be one of `"unspecified"` or `"don't_care"`.
   See @ref render_engine_default_label "RenderEngine" for more information. */
  std::string default_label{"unspecified"};

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};
};

}  // namespace geometry
}  // namespace drake
