#pragma once

#include "drake/common/drake_deprecated.h"
#include "drake/common/name_value.h"
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
    a->Visit(DRAKE_NVP(default_label));
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
  }

  /** Default render label to apply to a geometry when none is otherwise
   specified.  */
  render::RenderLabel default_label{render::RenderLabel::kUnspecified};

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};
};

namespace render {

using RenderEngineGlParams
    DRAKE_DEPRECATED("2023-08-01", "Use the geometry namespace instead.")
    = geometry::RenderEngineGlParams;

}  // namespace render

}  // namespace geometry
}  // namespace drake
