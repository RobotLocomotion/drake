#pragma once

#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/common/schema/transform.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/rgba.h"

namespace drake {
namespace systems {
namespace sensors {

/** Configuration of a camera. This covers all of the parameters for both
 color (see geometry::render::ColorRenderCamera) and depth (see
 geometry::render::DepthRenderCamera) cameras.

 The various properties have restrictions on what they can be.

   - Values must be finite.
   - Some values must be positive (see notes on individual properties).
   - Ranges must be specified such that the "minimum" value is less than or
     equal to the "maximum" value. This includes the clipping range
     [`clipping_near`, `clipping_far`] and depth range [`z_near`, `z_far`].
   - The depth range must lie *entirely* within the clipping range.

 The values are only checked when the configuration is operated on: during
 serialization, after deserialization, and when applying the configuration (see
 ApplyCameraConfig().) */
struct CameraConfig {
  /** Passes this object to an Archive.
   Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(width));
    a->Visit(DRAKE_NVP(height));
    a->Visit(DRAKE_NVP(focal));
    a->Visit(DRAKE_NVP(center_x));
    a->Visit(DRAKE_NVP(center_y));
    a->Visit(DRAKE_NVP(clipping_near));
    a->Visit(DRAKE_NVP(clipping_far));
    a->Visit(DRAKE_NVP(z_near));
    a->Visit(DRAKE_NVP(z_far));
    a->Visit(DRAKE_NVP(X_PB));
    a->Visit(DRAKE_NVP(X_BC));
    a->Visit(DRAKE_NVP(X_BD));
    a->Visit(DRAKE_NVP(renderer_name));
    a->Visit(DRAKE_NVP(background));
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(fps));
    a->Visit(DRAKE_NVP(rgb));
    a->Visit(DRAKE_NVP(depth));
    a->Visit(DRAKE_NVP(show_rgb));
    a->Visit(DRAKE_NVP(do_compress));
    ValidateOrThrow();
  }

  /** Specification of a camera's intrinsic focal properties as focal *length*
   (in pixels). One or both values can be given. When only one value is given,
   the other is assumed to match. At least one value must be provided. */
  struct FocalLength {
    /** Passes this object to an Archive.
     Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(x));
      a->Visit(DRAKE_NVP(y));
      ValidateOrThrow();
    }

    /** If specified, the focal length along this axis; otherwise, use
     focal length in the y-direction. */
    std::optional<double> x;

    /** If specified, the focal length along this axis; otherwise, use
     focal length in the x-direction. */
    std::optional<double> y;

    void ValidateOrThrow() const;

    /** Reports focal length along x-axis.
     @throws std::exception if both `x` and `y` are null. */
    double focal_x() const;

    /** Resolves focal length along y-axis.
     @throws std::exception if both `x` and `y` are null. */
    double focal_y() const;
  };

  /** Specification of focal length via fields of view (in degrees). */
  struct FovDegrees {
    /** Passes this object to an Archive.
     Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(x));
      a->Visit(DRAKE_NVP(y));
      ValidateOrThrow();
    }

    /** If specified, compute focal length along this axis; otherwise, use
     focal length given computation for `y`. */
    std::optional<double> x;

    /** If specified, compute focal length along this axis; otherwise, use
     focal length given computation for `x`. */
    std::optional<double> y;

    void ValidateOrThrow() const;

    /** Resolves focal length along x-axis based on image dimensions and defined
     value for `x` and/or `y`.
     @throws std::exception if both `x` and `y` are null. */
    double focal_x(int width, int height) const;

    /** Resolves focal length along y-axis based on image dimensions and defined
     value for `y` and/or `x`.
     @throws std::exception if both `x` and `y` are null. */
    double focal_y(int width, int height) const;
  };

  /** @name Camera intrinsics
   See CameraInfo.
   */
  //@{

  /** %Image width (in pixels). @pre width > 0. */
  int width{640};

  /** %Image height (in pixels). @pre height > 0. */
  int height{480};

  /** The focal properties of the camera. It can be specified in one of two
   ways:

     - A FocalLength which specifies the focal lengths (in pixels) in the x-
       and y-directions.
     - An FovDegrees which defines focal length *implicitly* by deriving it from
       field of view measures (in degrees).

   For both specifications, if only one value is given (in either direction),
   the focal length (as reported by focal_x() and focal_y()) is determined by
   that single value and is assumed to be symmetric for both directions.
   @pre focal length(s) is a/are finite, positive number(s). */
  std::variant<FocalLength, FovDegrees> focal{FovDegrees{.y = 45}};

  /** Returns the focal length (in pixels) in the x-direction. */
  double focal_x() const;

  /** Returns the focal length (in pixels) in the y-direction. */
  double focal_y() const;

  /** The x-position of the principal point (in pixels). To query what the
   current value is, use principal_point().
   @pre 0 < center_x < width or is std::nullopt. */
  std::optional<double> center_x{};

  /** The y-position of the principal point (in pixels). To query what the
   current value is, use principal_point().
   @pre 0 < center_y < height or is std::nullopt. */
  std::optional<double> center_y{};

  /** Returns the position of the principal point. This respects the semantics
   that undefined center_x and center_y place the principal point in the center
   of the image. */
  Vector2<double> principal_point() const {
    // This calculation should be kept in agreement with CameraInfo's.
    return {center_x.has_value() ? *center_x : width * 0.5 - 0.5,
            center_y.has_value() ? *center_y : height * 0.5 - 0.5};
  }

  //@}

  /** @name Frustum-based camera properties
   See geometry::render::ClippingRange.
   */
  //@{

  /** The distance (in meters) from sensor origin to near clipping plane.
   @pre clipping_near is a positive, finite number. */
  double clipping_near{0.01};

  /** The distance (in meters) from sensor origin to far clipping plane.
   @pre clipping_far is a positive, finite number. */
  double clipping_far{500.0};

  //@}

  /** @name Depth camera range
   See geometry::render::DepthRange.
   */
  //@{

  /** The distance (in meters) from sensor origin to closest measurable plane.
   @pre z_near is a positive, finite number. */
  double z_near{0.1};

  /** The distance (in meters) from sensor origin to farthest measurable plane.
   @pre z_near is a positive, finite number. */
  double z_far{5.0};

  //@}

  /** @name Camera extrinsics
   The pose of the camera.

   As documented in RgbdSensor, the camera has four associated frames: P, B, C,
   D. The frames of the parent (to which the camera body is rigidly affixed),
   the camera body, color sensor, and depth sensor, respectively. Typically, the
   body is posed w.r.t. the parent (X_PB) and the sensors are posed w.r.t. the
   body (X_BC and X_BD).

   When unspecified, X_BD = X_BC = I by default. */
  //@{

  /** The pose of the body camera relative to a parent frame P. If
   `X_PB.base_frame` is unspecified, then the world frame is assumed to be
   the parent frame.
   @pre `X_PB.base_frame` is empty *or* refers to a valid, unique frame. */
  schema::Transform X_PB;

  /** The pose of the color sensor relative to the camera body frame.
   @pre `X_BC.base_frame` is empty. */
  schema::Transform X_BC;

  /** The pose of the depth sensor relative to the camera body frame.
   @pre `X_BD.base_frame` is empty. */
  schema::Transform X_BD;

  //@}

  /** @name Renderer properties

   Every camera is supported by a geometry::render::RenderEngine
   instance. These properties configure the render engine for this camera. */
  //@{

  /** The name of the geometry::render::RenderEngine that this camera
   uses. Generally, it is more efficient for multiple cameras to share a single
   render engine instance; they should, therefore, share a common renderer_name
   value.
   @pre `renderer_name` is not empty. */
  std::string renderer_name{"default"};

  /** The "background" color. This is the color drawn where there are no objects
   visible. Its default value matches the default value for
   render::RenderEngineGlParams::default_clear_color. See the
   documentation for geometry::Rgba::Serialize for how to define this
   value in YAML.

   N.B. If two different cameras are configured to use the same renderer (having
   identical values for `renderer_name`) but *different* values for
   `background`, the render engine instance will be configured with one of the
   set of values. Which one is undefined. */
  geometry::Rgba background{204 / 255.0, 229 / 255.0, 255 / 255.0, 1.0};

  //@}

  /** @name Publishing properties */
  //@{

  /** The camera name. This `name` is used as part of the corresponding
   RgbdSensor system's name and serves as a suffix of the LCM channel name.
   @pre `name` is not empty.*/
  std::string name{"preview_camera"};

  /** Publishing rate (in Hz) for both RGB and depth cameras (as requested).
   @pre fps is a positive, finite number. */
  double fps{10.0};

  /** If true, RGB images will be produced and published via LCM. */
  bool rgb{true};

  /** If true, depth images will be produced and published via LCM. */
  bool depth{false};

  /** Controls whether the rendered RGB images are displayed (in a separate
   window controlled by the thread in which the camera images are rendered).
   Only applies to color images and depends on whether the RenderEngine instance
   supports it. */
  bool show_rgb{false};

  /** Controls whether the images are broadcast in a compressed format. */
  bool do_compress{true};

  //@}

  /** Creates color and depth camera data from this configuration.
   @throws std::exception if configuration values do not satisfy the documented
                          prerequisites. */
  std::pair<geometry::render::ColorRenderCamera,
            geometry::render::DepthRenderCamera>
  MakeCameras() const;

  /** Throws if the values are inconsistent. */
  void ValidateOrThrow() const;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
