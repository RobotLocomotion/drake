#pragma once

#include <optional>
#include <vector>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/geometry/render/light_parameter.h"
#include "drake/geometry/render/render_label.h"

namespace drake {
namespace geometry {

/** Construction parameters for the RenderEngineVtk.  */
struct RenderEngineVtkParams  {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(default_diffuse));
    a->Visit(DRAKE_NVP(default_clear_color));
    a->Visit(DRAKE_NVP(lights));
  }

  /** (Deprecated.) The default_label is no longer configurable. <br>
   This will be removed from Drake on or after 2023-12-01. */
  std::optional<render::RenderLabel> default_label;

  /** The (optional) rgba color to apply to the (phong, diffuse) property when
    none is otherwise specified. Note: currently the alpha channel is unused
    by RenderEngineVtk.  */
  std::optional<Eigen::Vector4d> default_diffuse{};

  /** The rgb color to which the color buffer is cleared (each
   channel in the range [0, 1]). The default value (in byte values) would be
   [204, 229, 255].  */
  Eigen::Vector3d default_clear_color{204 / 255., 229 / 255., 255 / 255.};

  /** Lights in the scene. More than five lights is an error. If no lights are
   defined, a single directional light, fixed to the camera frame, is used.

   Note: RenderEngineVtk does not have a hard-coded limit on the number of
         lights; but more lights increases rendering cost.
   Note: the attenuation values have no effect on VTK *directional* lights. */
  std::vector<render::LightParameter> lights;
};

}  // namespace geometry
}  // namespace drake
