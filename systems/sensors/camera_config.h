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
 ApplyCameraConfig().)

 <h3>Cameras and RenderEngines</h3>

 @anchor camera_config_render_properties

 Every camera is supported by a geometry::render::RenderEngine instance. These
 properties configure the render engine for this camera.

 RenderEngines are uniquely identified by their name (as specified by
 `renderer_name`) and configured by `renderer_class`. Each RenderEngine instance
 must have a unique name. Rendering will be more efficient if multiple cameras
 share the same RenderEngine instance. To share RenderEngine instances, cameras
 must have _same_ value for `renderer_name`.

 Each camera can also provide the configuration of its supporting RenderEngine
 (`renderer_class`). However, it is an error for two cameras to specify the same
 `renderer_name` but provide _conflicting_ renderer configurations. It is the
 _user's_ responsibility to make sure that for every shared `renderer_name`
 value in a set of %CameraConfig instances that the corresponding
 `renderer_class` values are compatible. How that is achieved depends on the
 medium of specification. While there are multiple possible mechanisms, these
 examples consist of the simplest.

 <h4>Configuring compatible `renderer_class` in YAML</h4>

 The simplest solution in YAML is to use the merge operator (`<<:`) to guarantee
 that multiple cameras use the exact same `renderer_class`. This terse example
 shows how that might be done.

 @code{.yaml}
 cameras:
   - &BaseCamera:
     name: base_camera
     rgb: true
     depth: true
     label: true
     renderer_name: common_renderer
     renderer_class: !RenderEngineVtk
       exposure: 0.4
       cast_shadows: true
   - &DepthOnlyCamera
     <<: *BaseCamera
     name: depth_camera
     rgb: false
     label: false
 @endcode

 In this example, we've defined two cameras named, `base_camera` and
 `depth_camera`. Both share a RenderEngineVtk instance named `common_renderer`.
 The merge operator guarantees that `depth_camera` has exactly duplicated
 `base_camera`'s `renderer_class` value. However, it tweaks the camera
 definition by disabling the rgb and label images. Edits to the
 `RenderEngineVtk` parameters in a single location are kept in sync across all
 cameras that are supposed to share.

 <h4>Configuring compatible `renderer_class` in C++</h4>

 The same trick for coordinating multiple %CameraConfig instances in yaml will
 also work in C++. Simply instantiate a single set of RenderEngine parameters
 and assign it to every instance.

 In C++ there are other options available. If you have control over the order
 that %CameraConfig instances get applied, you can simply define the
 `renderer_class` value for the first instance and leave it blank in subsequent
 instances which share the same `renderer_name`. It is important to configure
 the RenderEngine on the _first_ appearance of a `renderer_name` value.

 @anchor camera_config_compatible_renderer
 <h4>Rules for compatible `renderer_class` values</h4>

 Assume we have two %CameraConfig instances that have a common `renderer_name`
 value (and both have well-formed `renderer_class` values). When we apply the
 first config instance, we will add a RenderEngine. When we attempt to apply
 the second config instance, we already have a RenderEngine with the specified
 name. Compatibility now depends on the `renderer_class` value stored in the
 second config instance. It will be compatible in the following cases:

   - The second config instance's `renderer_class` value is the empty string.
   - The second config instance's `renderer_class` value is the *class name* of
     the RenderEngine instance that was already created.
   - The second config instance's `renderer_class` contains the appropriate
     parameters type for the type of RenderEngine instance that was already
     created _and_ the parameter values _exactly_ match those in the
     instantiated engine.

 Every other `renderer_class` value in the second config instance (e.g., naming
 a different type of RenderEngine, or specifying different parameters) is
 considered a conflicting specification and will give rise to runtime errors. */
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

  /** @name Renderer properties */
  //@{

  /** The name of the geometry::render::RenderEngine that this camera
   uses.
   @pre `renderer_name` is not empty. */
  std::string renderer_name{"default"};

  /** The configuration of the camera's supporting RenderEngine.

   The value can be one of:
      - An empty string.
      - A string containing the class name of the RenderEngine to use (i.e.,
        `"RenderEngineVtk"`, `"RenderEngineGl"`, or `"RenderEngineGltfClient"`).
      - The struct of parameters for a supported engine, i.e.,
        RenderEngineVtkParams, RenderEngineGlParams, or
        RenderEngineGltfClientParams.

   For config instances which have a unique `renderer_name` value (one that has
   not yet been added to the diagram), the various possible `renderer_class`
   values will instantiate a new RenderEngine and associate it with a camera
   according to the following rules.

      - Empty string: a RenderEngine of the _default_ type (RenderEngineVtk)
        will be instantiated with _default_ parameters.
      - Class name string: a RenderEngine of the _named_ type will be
        instantiated with _default_ parameters.
      - Parameter struct: a RenderEngine of the type compatible with the struct
        will be instantiated, with those parameter values.

   If, however, the `renderer_name` is not unique, as documented
   @ref camera_config_compatible_renderer "above", the values of
   `renderer_class` must be _compatible_ with each other.

   Note: RenderEngineVtk is the default RenderEngine type. It is slower, but
   portable and robust. RenderEngineGl is an option if you are on Ubuntu and
   need the improved performance (at the *possible* cost of lesser image
   fidelity).

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

   6. Explicitly request Drake's default render engine or previously configured
      RenderEngine based on shared `renderer_name`.
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
   specific render engine and its configuration (see e.g.,
   geometry::RenderEngineVtkParams::backend).

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
