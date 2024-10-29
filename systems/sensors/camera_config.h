#pragma once

#include <optional>
#include <string>
#include <utility>
#include <variant>

#include "drake/common/eigen_types.h"
#include "drake/common/name_value.h"
#include "drake/common/schema/transform.h"
#include "drake/geometry/render/render_camera.h"
#include "drake/geometry/render_gl/render_engine_gl_params.h"
#include "drake/geometry/render_gltf_client/render_engine_gltf_client_params.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
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
    a->Visit(DRAKE_NVP(renderer_class));
    a->Visit(DRAKE_NVP(background));
    a->Visit(DRAKE_NVP(name));
    a->Visit(DRAKE_NVP(fps));
    a->Visit(DRAKE_NVP(capture_offset));
    a->Visit(DRAKE_NVP(output_delay));
    a->Visit(DRAKE_NVP(rgb));
    a->Visit(DRAKE_NVP(depth));
    a->Visit(DRAKE_NVP(label));
    a->Visit(DRAKE_NVP(show_rgb));
    a->Visit(DRAKE_NVP(do_compress));
    a->Visit(DRAKE_NVP(lcm_bus));
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

  /** The choice of render engine implementation to use. It can be specified
   simply by providing a `string` containing the class _name_ of the
   RenderEngine to use (i.e., `"RenderEngineVtk"`, `"RenderEngineGl"`, or
   `"RenderEngineGltfClient"`). Or, it can be specified by providing parameters
   for one of those engines: RenderEngineVtkParams, RenderEngineGlParams, or
   RenderEngineGltfClientParams.

   If a `string` containing the render engine class _name_ is provided, the
   engine instantiated will use the default parameters -- equivalent to passing
   the set of default parameters.

   It is possible for multiple cameras to reference the same `renderer_name`
   but configure the renderer differently. This would be a configuration error.
   The following rules will help prevent problematic configurations:

     - Multiple cameras can reference the same value for `renderer_name`.
     - If multiple cameras reference the same value for `renderer_name`, only
       the *first* can use the engine parameters to specify it. Attempting to
       do so with a later camera will produce an error.
       - The later cameras can use engine class _name_.
       - If a later camera names a *different* engine class, that will result
         in an error.

   In YAML, it can be a bit trickier. Depending on how a collection of cameras
   is articulated, the concept of "first" may be unclear. In
   `examples/hardware_sim/scenario.h` the collection is a map. So, the camera
   configurations are not necessarily processed in the order they appear in the
   YAML file. Instead, the processing order depends on the mnemonic camera key.
   So, be aware, if you use a similar mapping, you may have to massage the key
   names to achieve the requisite processing order. Alternatively, a vector of
   %CameraConfig in your own scenario file, would guarantee that processing
   order is the same as file order.

   We intend to relax these restrictions in time, allowing equivalent, redundant
   specifications of a render engine (and throwing only on inconsistent
   specifications).

   Passing the empty string is equivalent to saying, "I don't care". If a render
   engine with that name has already been configured, the camera will use it
   (regardless of type). If the name doesn't already exist, the default, slower,
   more portable, and more robust RenderEngineVtk will be instantiated.
   RenderEngineGl can be selected if you are on Ubuntu and need the improved
   performance (at the *possible* cost of lesser image fidelity). In YAML,
   omitting `renderer_class` from the camera specification is equivalent to
   "passing the empty string".

   <h4>Configuring in YAML</h4>

   It isn't always obvious what the proper spelling in YAML is. The following
   examples illustrate how the `renderer_class` field can be defined in YAML
   files.

   1. Use the RenderEngineVtk with all default parameters - providing the class
      name as a string.
   @code{yaml}
   renderer_class: RenderEngineVtk
   @endcode

   2. Use the RenderEngineVtk with all default parameters - providing the
      engine parameters with default values.
   @code{yaml}
   renderer_class: !RenderEngineVtkParams {}
   @endcode

   3. Use the RenderEngineVtk with a customized clear color (black).
   @code{yaml}
   renderer_class: !RenderEngineVtkParams
     default_clear_color: [0, 0, 0]
   @endcode

   4. Use the RenderEngineGl with a customized clear color (black).
   @code{yaml}
   renderer_class: !RenderEngineGlParams
     default_clear_color:
       rgba: [0, 0, 0, 1]
   @endcode

   5. Use the RenderEngineGltfClient with fully specified properties.
   @code{yaml}
   renderer_class: !RenderEngineGltfClientParams
     base_url: http://10.10.10.1
     render_endpoint: server
     verbose: true
     cleanup: false
   @endcode

   6. Explicitly request Drake's default render engine.
   @code{yaml}
   renderer_class: ""
   @endcode

   Things to note:

     - When providing the _parameters_ for the engine, the declaration must
       begin with `!` to announce it as a type (examples 2, 3, and 4).
     - A defaulted set of parameters must have a trailing `{}` (example 2).
     - Two engine parameter sets may have the same semantic parameter but spell
       it differently (`default_clear_color` in examples 3 and 4).

   @pre `renderer_class` is a string and either empty or one of
        `"RenderEngineVtk"`, `"RenderEngineGl"`, or `"RenderEngineGltfClient"`,
        or, it is a parameter type of one of Drake's RenderEngine
        implementations.
   @sa drake::geometry::SceneGraph::GetRendererTypeName(). */
  std::variant<std::string, geometry::RenderEngineVtkParams,
               geometry::RenderEngineGlParams,
               geometry::RenderEngineGltfClientParams>
      renderer_class{""};

  /** The "background" color. This is the color drawn where there are no objects
   visible. Its default value matches the default value for
   render::RenderEngineVtkParams::default_clear_color. See the
   documentation for geometry::Rgba::Serialize for how to define this
   value in YAML.

   This value is used only if the `render_class` specifies either
   `"RenderEngineVtk"` or `"RenderEngineGl"` by _name_ (RenderEngineGltfClient
   doesn't have a configurable background color.) */
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

  /** Phase offset (in seconds) for image capture, relative to the simulator's
   time zero. This can be useful to stagger multiple cameras.
   Refer to the RgbdSensorAsync class for a comprehensive description.
   @pre capture_offset is non-negative and finite. */
  double capture_offset{0.0};

  /** Delay (in seconds) between when the scene graph geometry is "captured"
   and when the output image is published. Refer to the RgbdSensorAsync class
   for a comprehensive description.
   @pre output_delay is non-negative and strictly less than 1/fps. */
  double output_delay{0.0};

  /** If true, RGB images will be produced and published via LCM. */
  bool rgb{true};

  /** If true, depth images will be produced and published via LCM. */
  bool depth{false};

  /** If true, label images will be produced and published via LCM. */
  bool label{false};

  /** Controls whether the rendered RGB and/or label images are displayed (in
   separate windows controlled by the thread in which the camera images are
   rendered). Because both RGB and label images are configured from the same
   `ColorRenderCamera`, this setting applies to both images. Even when set to
   true, whether or not the image is able to be displayed depends on the
   specific render engine its configuration.

   Note: This flag is intended for quick debug use during development instead of
   serving as an image viewer. Currently, there are known issues, e.g.,
   flickering windows when multiple cameras share the same renderer or
   upside-down images if RenderEngineGl is set. See issue #18862 for the
   proposal to visualize images via Meldis. */
  bool show_rgb{false};

  /** Controls whether the images are broadcast in a compressed format. */
  bool do_compress{true};

  /** Which LCM URL to use.
  @see drake::systems::lcm::LcmBuses */
  std::string lcm_bus{"default"};

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
