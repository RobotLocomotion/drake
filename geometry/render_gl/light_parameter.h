#pragma once

#include <Eigen/Dense>

namespace drake {
namespace geometry {
namespace render {

enum LightType {
  POINT = 1,
  SPOTLIGHT = 2,
  DIRECTIONAL = 3
};

/** Light parameter for RenderEngineGlParams.  */
struct LightParameter {
  /** The light type is either Point, Spotlight, or Directional.
    Default is Directional light, because that is what it was in VTK.  */
  LightType type{LightType::DIRECTIONAL};
  /** The cone angle for a positional light in degrees.  */
  double cone_angle{0};
  /** The quadratic attenuation constants. They are specified as constant,
    linear, and quadratic, in that order.  */
  Eigen::Vector3d attenuation_values = {0, 0, 0};
  /** The position of the light.  */
  Eigen::Vector3d position = {0, 0, 0};
  /** The brightness of the light (from one to zero).  */
  double intensity{0};
  /** Default light direction is point down. Only used for directional
    lights  */
  Eigen::Vector3d light_direction = {0, 0, 1};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake
