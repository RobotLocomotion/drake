#pragma once

#include "drake/geometry/render/render_label.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace render {

/** Construction parameters for RenderEngineGl.  */
struct RenderEngineGlParams {
  /** Default render label to apply to a geometry when none is otherwise
   specified.  */
  RenderLabel default_label{RenderLabel::kUnspecified};

  /** Default diffuse color to apply to a geometry when none is otherwise
   specified in the (phong, diffuse) property.  */
  Rgba default_diffuse{0.9, 0.7, 0.2, 1.0};

  /** The default background color for color images.  */
  Rgba default_clear_color{204 / 255., 229 / 255., 255 / 255., 1.0};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
