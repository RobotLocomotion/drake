#pragma once

#include <optional>

#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams  {
  /** The (optional) label to apply when none is otherwise specified.  */
  std::optional<render::RenderLabel> default_label{};

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  /** The rgb color to which the color buffer is cleared (each
   channel in the range [0, 1]). The default value (in byte values) would be
   [204, 229, 255].  */
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};
};

namespace render {

using RenderEngineVtkParams
    DRAKE_DEPRECATED("2022-09-01", "Use the geometry namespace instead.")
    = geometry::RenderEngineVtkParams;

}  // namespace render
}  // namespace geometry
}  // namespace drake
