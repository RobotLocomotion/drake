#pragma once

#include <string>
#include <string_view>

#include <Eigen/Dense>
#include <fmt/format.h>

#include "drake/common/fmt.h"
#include "drake/common/name_value.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace geometry {
namespace render {

/** Specification of the type of light.

  - kPoint: a punctual light source emitting light in all directions.
  - kSpot: A conical light source emitting light from a point into a limited
           set of directions.
  - kDirectional: a light source, infinitely far away, casting parallel rays of
                  light (like a sun). */
enum class LightType { kPoint = 1, kSpot = 2, kDirectional = 3 };

/** Returns the LightType as a string. */
std::string_view to_string(const LightType& t);

/** Instantiates a LightType from its string representation.
 @param spec  Must be one of 'point', 'spot', or 'directional'.
 @throws if `spec` is an unrecognized string. */
LightType light_type_from_string(const std::string& spec);

// TODO(SeanCurtis-TRI): Allow fixing the camera to an arbitrary frame.

/** Specifies the frame in which a light is fixed.

 Fixing the camera to the world keeps it stationary. Fixing it to the camera
 moves it with the camera. */
enum class LightFrame { kWorld = 0, kCamera = 1 };

/** Returns the LightFrame as a string. */
std::string_view to_string(const LightFrame& f);

/** Instantiates a LightFrame from its string representation.
 @param spec  Must be one of 'world' or 'camera'.
 @throws if `spec` is an unrecognized string. */
LightFrame light_frame_from_string(const std::string& spec);

/** Light parameter for supporting RenderEngine implementations. Look at the
 various RenderEngine___Params types to see if they support declaring lights. */
struct LightParameter {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(type));
    a->Visit(DRAKE_NVP(color));
    a->Visit(DRAKE_NVP(attenuation_values));
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(frame));
    a->Visit(DRAKE_NVP(intensity));
    a->Visit(DRAKE_NVP(direction));
    a->Visit(DRAKE_NVP(cone_angle));
  }

  /** The light type is either `"point"`, `"spot"`, or `"directional"`. */
  std::string type{"directional"};

  /** The illuminating color of the light (with channels in the range [0, 1]).
    The alpha value is currently ignored. */
  Rgba color = Rgba(1, 1, 1);

  /** The quadratic attenuation constants (k₀, k₁, and k₂). The intensity of
   light can decrease with distance according to the following equation:

      I = 1/(k₀ + k₁·d + k₂·d²)

   In the physical world, light from a point source decreases with the squared
   distance (with some linear attenuation due to atmospheric effects). However,
   this often leads to undesirable illumination effects in low-dynamic range
   rendering. Suitable values typically will temper the quadratic effects with
   non-zero constant and linear terms.

   If the values sum to 1, then, a surface one meter away from the light will
   get full illumination (surfaces closer will get *magnified* illumination.
   When tuning attenuation to achieve a particular falloff *pattern*, it is
   common to *increase* the light `intensity` to offset what would otherwise
   appear to be a rapid decay. By default, the light is unattenuated, with
   constant illumination across all distances.

   When using directional lights, attenuation is best kept with the default,
   non attenuated values (<1, 0, 0>). Other attenuation values may be applied,
   but the effect may be surprising and difficult to control.

   @warning If all three values are zero, no light is emitted. */
  Eigen::Vector3d attenuation_values = {1, 0, 0};

  /** The position of the light in the frame indicated by `frame` F: p_FL. */
  Eigen::Vector3d position{0, 0, 0};

  /** Specifies the frame to which the camera is fixed, `"world"` or `"camera"`.

   Remember, as documented by @ref systems::sensors::CameraInfo "CameraInfo",
   the camera frame has +Cz pointing *into* the image, +Cx pointing to the
   right, and +Cy pointing to the bottom of the image. */
  std::string frame{"camera"};

  /** A multiplier for the brightness of the light. A zero intensity will
   effectively disable the light. A value of one will have an intensity equal to
   the light's color. Higher values will magnify the light's illumination (and
   may be necessary to offset the attenuation effects). */
  double intensity{1};

  /** The direction the light points in the frame indicated by `frame`. For
   example, in the world frame, a spotlight with direction [0, 0, -1]ᵀ points
   straight down. In the camera frame, the direction [0, 0, 1]ᵀ points forward
   in the camera's view direction. This field only applies to spotlight and
   directional lights.

   @pre the vector has sufficient precision to be meaningfully normalized. */
  Eigen::Vector3d direction{0, 0, 1};

  /** For a spotlight, it is the measure of the angle (in degrees) between the
   light's central direction vector and the border of the light's conical
   extent. It is half of the spotlight's full angular span. So, a light with a
   45° `cone_angle` would cast light over a 90° span. */
  double cone_angle{0};
};

}  // namespace render
}  // namespace geometry
}  // namespace drake

DRAKE_FORMATTER_AS(, drake::geometry::render, LightType, x,
                   drake::geometry::render::to_string(x))
DRAKE_FORMATTER_AS(, drake::geometry::render, LightFrame, x,
                   drake::geometry::render::to_string(x))
