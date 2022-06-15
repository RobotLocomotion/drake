#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
// TODO(russt): Move point_cloud.h to a more central location.
#include "drake/perception/point_cloud.h"

namespace drake {
namespace geometry {

/** The set of parameters for configuring Meshcat. */
struct MeshcatParams {
  /** Passes this object to an Archive.
  Refer to @ref yaml_serialization "YAML Serialization" for background. */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(host));
    a->Visit(DRAKE_NVP(port));
    a->Visit(DRAKE_NVP(web_url_pattern));
    a->Visit(DRAKE_NVP(show_stats_plot));
  }

  /** Meshcat will listen only on the given hostname (e.g., "localhost").
  If "*" is specified, then it will listen on all interfaces.
  If empty, an appropriate default value will be chosen (currently "*"). */
  std::string host{"*"};

  /** Meshcat will listen on the given http `port`. If no port is specified,
  then it will listen on the first available port starting at 7000 (up to 7099).
  @pre We require `port` >= 1024. */
  std::optional<int> port{std::nullopt};

  /** The `web_url_pattern` may be used to change the web_url() (and therefore
  the ws_url()) reported by Meshcat. This may be useful in case %Meshcat sits
  behind a firewall or proxy.

  The pattern follows the
  <a href="https://en.cppreference.com/w/cpp/utility/format">std::format</a>
  specification language, except that `arg-id` substitutions are performed
  using named arguments instead of positional indices.

  There are two arguments available to the pattern:
  - `{port}` will be substituted with the %Meshcat server's listen port number;
  - `{host}` will be substituted with this params structure's `host` field, or
    else with "localhost" in case the `host` was one of the placeholders for
    "all interfaces".
  */
  std::string web_url_pattern{"http://{host}:{port}"};

  /** Determines whether or not to display the stats plot widget in the Meshcat
  user interface. This plot including realtime rate and WebGL render
  statistics. */
  bool show_stats_plot{true};
};

/** Provides an interface to %Meshcat (https://github.com/rdeits/meshcat).

Each instance of this class spawns a thread which runs an http/websocket server.
Users can navigate their browser to the hosted URL to visualize the Meshcat
scene.  Note that, unlike many visualizers, one cannot open the visualizer until
this server is running.

@warning In the current implementation, Meshcat methods must be called from the
same thread where the class instance was constructed.  For example, running
multiple simulations in parallel using the same Meshcat instance is not yet
supported. We may generalize this in the future.

@section meshcat_path Meshcat paths and the scene tree

https://github.com/rdeits/meshcat#api provides a nice introduction to the
websocket API that we wrap with this class.  One of the core concepts is the
"scene tree" -- a directory-like structure of objects, transforms, and
properties. The scene tree is viewable in the browser by clicking on "Open
Controls" in the top right corner.

Elements of the tree are referenced programmatically by a "/"-delimited string
indicating the object's **path** in the scene tree. An object at path "/foo/bar"
is a child of an object at path "/foo", so setting the transform of (or
deleting) "/foo" will also affect its children.

The string path arguments to the methods in this class use the following
semantics:
- A path that begins with "/" is treated as an absolute path, and is used
without modification.
- A path that *does not* begin with "/" is treated as a relative path to the
default working directory.
- The "working directory" is fixed to be "/drake" in the current implementation.
So any relative path "foo" will be treated as "/drake/foo".
- Delete("/foo") will remove all objects, transforms, and properties in "/foo"
and its children from the scene tree.
- Delete() is equivalent to Delete("") or Delete("/drake/").
- SetObject("/foo", ...) places the object at "/foo/<object>", where "<object>"
is a hard-coded suffix used for all objects.  You can use the
Delete("/foo/<object>") to delete the object only (and not the children of
"/foo") and must use SetProperty("/foo/<object>", ...) to change object-specific
properties.

The root directory contains a number of elements that are set up automatically
in the browser.  These include "/Background", "/Lights", "/Grid", and
"/Cameras".  To find more details please see the @link
https://github.com/rdeits/meshcat#api meshcat documentation @endlink and the
@link https://threejs.org/docs/index.html three.js documentation @endlink.
- You can modify these elements, create new lights/cameras, and even delete
these elements (one at a time).
- Delete("/") is not allowed.  It will be silently ignored.

@section meshcat_recommended_workflow Recommended workflow

For convenience, Meshcat fosters a work flow in which all user-created objects
created in Drake are contained in the "/drake" folder. Objects added with a
relative path are placed in the "/drake" folder for you. The benefits are:
- It's simple to distinguish between user objects and "infrastructure" objects
in the visualizer.
- All user objects can easily be cleared by a single, parameter-free call to
Delete(). You are welcome to use absolute paths to organize your data, but the
burden on tracking and cleaning them up lie on you.
*/
class Meshcat {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Meshcat)

  /** Constructs the %Meshcat instance on `port`. If no port is specified,
  it will listen on the first available port starting at 7000 (up to 7099).
  @pre We require `port` >= 1024.
  @throws std::exception if no requested `port` is available. */
  explicit Meshcat(std::optional<int> port = std::nullopt);

  /** Constructs the %Meshcat instance using the given `params`. */
  explicit Meshcat(const MeshcatParams& params);

  ~Meshcat();

  /** Returns the hosted http URL. */
  std::string web_url() const;

  /** Returns the port on localhost listening for http connections. */
  int port() const;

  /** (Advanced) Returns the ws:// URL for direct connection to the websocket
  interface.  Most users should connect via a browser opened to web_url(). */
  std::string ws_url() const;

  /** (Advanced) Returns the number of currently-open websocket connections. */
  int GetNumActiveConnections() const;

  /** Blocks the calling thread until all buffered data in the websocket thread
  has been sent to any connected clients. This can be especially useful when
  sending many or large mesh files / texture maps, to avoid large "backpressure"
  and/or simply to make sure that the simulation does not get far ahead of the
  visualization. */
  void Flush() const;

  /** Sets the 3D object at a given `path` in the scene tree.  Note that
  `path`="/foo" will always set an object in the tree at "/foo/<object>".  See
  @ref meshcat_path.  Any objects previously set at this `path` will be
  replaced.
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path "Meshcat paths" for the semantics.
  @param shape a Shape that specifies the geometry of the object.
  @param rgba an Rgba that specifies the (solid) color of the object.
  @pydrake_mkdoc_identifier{shape}
  */
  void SetObject(std::string_view path, const Shape& shape,
                 const Rgba& rgba = Rgba(.9, .9, .9, 1.));

  // TODO(russt): SetObject with texture map.

  /** Sets the "object" at a given `path` in the scene tree to be
  `point_cloud`.  Note that `path`="/foo" will always set an object in the tree
  at "/foo/<object>".  See @ref meshcat_path.  Any objects previously set at
  this `path` will be replaced.
  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param point_cloud a perception::PointCloud; if `point_cloud.has_rgbs()` is
                     true, then meshcat will render the colored points.
  @param point_size is the size of each rendered point.
  @param rgba is the default color, which is only used if
              `point_cloud.has_rgbs() == false`.
  @pydrake_mkdoc_identifier{cloud}
  */
  void SetObject(std::string_view path,
                 const perception::PointCloud& point_cloud,
                 double point_size = 0.001,
                 const Rgba& rgba = Rgba(.9, .9, .9, 1.));

  /** Sets the "object" at `path` in the scene tree to a TriangleSurfaceMesh.

  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param mesh is a TriangleSurfaceMesh object.
  @param rgba is the mesh face or wireframe color.
  @param wireframe if "true", then only the triangle edges are visualized, not
                   the faces.
  @param wireframe_line_width is the width in pixels.  Due to limitations in
                              WebGL implementations, the line width may be 1
                              regardless of the set value.
  @pydrake_mkdoc_identifier{triangle_surface_mesh}
  */
  void SetObject(std::string_view path, const TriangleSurfaceMesh<double>& mesh,
                 const Rgba& rgba = Rgba(0.1, 0.1, 0.1, 1.0),
                 bool wireframe = false, double wireframe_line_width = 1.0);

  /** Sets the "object" at `path` in the scene tree to a piecewise-linear
  interpolation between the `vertices`.

  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param vertices are the 3D points defining the lines.
  @param line_width is the width in pixels.  Due to limitations in WebGL
                    implementations, the line width may be 1 regardless
                    of the set value.
  @param rgba is the line color. */
  void SetLine(std::string_view path,
               const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
               double line_width = 1.0,
               const Rgba& rgba = Rgba(0.1, 0.1, 0.1, 1.0));

  /** Sets the "object" at `path` in the scene tree to a number of line
  segments.

  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param start is a 3-by-N matrix of 3D points defining the start of each
               segment.
  @param end is a 3-by-N matrix of 3D points defining the end of each
             segment.
  @param line_width is the width in pixels.  Due to limitations in WebGL
                    implementations, the line width may be 1 regardless
                    of the set value.
  @param rgba is the line color.

  @throws std::exception if start.cols != end.cols(). */
  void SetLineSegments(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& start,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& end,
                       double line_width = 1.0,
                       const Rgba& rgba = Rgba(0.1, 0.1, 0.1, 1.0));

  // TODO(russt): Support per-vertex coloring (maybe rgba as std::variant).
  /** Sets the "object" at `path` in the scene tree to a triangular mesh.

  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param vertices is a 3-by-N matrix of 3D point defining the vertices of the
                  mesh.
  @param faces is a 3-by-N integer matrix with each entry denoting an index
               into vertices and each column denoting one face (aka
               SurfaceTriangle).
  @param rgba is the mesh face or wireframe color.
  @param wireframe if "true", then only the triangle edges are visualized, not
                   the faces.
  @param wireframe_line_width is the width in pixels.  Due to limitations in
                              WebGL implementations, the line width may be 1
                              regardless of the set value. */
  void SetTriangleMesh(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                       const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                       const Rgba& rgba = Rgba(0.1, 0.1, 0.1, 1.0),
                       bool wireframe = false,
                       double wireframe_line_width = 1.0);

  void SetTriangleColorMesh(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                       const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& colors,
                       bool wireframe = false,
                       double wireframe_line_width = 1.0);

  // TODO(russt): Provide a more general SetObject(std::string_view path,
  // msgpack::object object) that would allow users to pass through anything
  // that meshcat.js / three.js can handle.  Possible this could use
  // common/name_value.h to avoid a header dependency on msgpack.

  /** Properties for a perspective camera in three.js:
   https://threejs.org/docs/#api/en/cameras/PerspectiveCamera */
  struct PerspectiveCamera {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(fov));
      a->Visit(DRAKE_NVP(aspect));
      a->Visit(DRAKE_NVP(near));
      a->Visit(DRAKE_NVP(far));
      a->Visit(DRAKE_NVP(zoom));
    }

    double fov{75};    ///< Camera frustum vertical field of view.
    double aspect{1};  ///< Camera frustum aspect ratio.
    double near{.01};  ///< Camera frustum near plane.
    double far{100};   ///< Camera frustum far plane.
    double zoom{1};    ///< The zoom factor of the camera.
  };

  /** Sets the Meshcat object on `path` to a perspective camera. We provide a
   default value of `path` corresponding to the default camera object in
   Meshcat.

   @pydrake_mkdoc_identifier{perspective}
   */
  void SetCamera(PerspectiveCamera camera,
                 std::string path = "/Cameras/default/rotated");

  /** Properties for an orthographic camera in three.js:
   https://threejs.org/docs/#api/en/cameras/OrthographicCamera */
  struct OrthographicCamera {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(left));
      a->Visit(DRAKE_NVP(right));
      a->Visit(DRAKE_NVP(top));
      a->Visit(DRAKE_NVP(bottom));
      a->Visit(DRAKE_NVP(near));
      a->Visit(DRAKE_NVP(far));
      a->Visit(DRAKE_NVP(zoom));
    }

    double left{-1};     ///< Camera frustum left plane.
    double right{1};     ///< Camera frustum right plane.
    double top{-1};      ///< Camera frustum top plane.
    double bottom{1};    ///< Camera frustum bottom plane.
    double near{-1000};  ///< Camera frustum near plane.
    double far{1000};    ///< Camera frustum far plane.
    double zoom{1};      ///< The zoom factor of the camera.
  };

  /** Sets the Meshcat object on `path` to an orthographic camera. We provide a
   default value of `path` corresponding to the default camera object in
   Meshcat.

   @pydrake_mkdoc_identifier{orthographic}
   */
  void SetCamera(OrthographicCamera camera,
                 std::string path = "/Cameras/default/rotated");

  /** Applies a number of settings to make Meshcat act as a 2D renderer. The
   camera is set to an orthographic camera with `X_WC` specifying the pose of
   the camera in world.  The camera looks down the +Cy axis, with +Cz
   corresponding to positive y in the 2D frame, and -Cx corresponding to
   positive x in the 2D frame.

   Additionally sets the background, grid lines, and axes "visible" properties
   to false. */
  void Set2dRenderMode(const math::RigidTransformd& X_WC =
                           math::RigidTransformd(Eigen::Vector3d{0, -1, 0}),
                       double xmin = -1, double xmax = 1, double ymin = -1,
                       double ymax = 1);

  /** Resets the default camera, background, grid lines, and axes to their
   default settings. */
  void ResetRenderMode();

  /** Set the RigidTransform for a given path in the scene tree relative to its
  parent path. An object's pose is the concatenation of all of the transforms
  along its path, so setting the transform of "/foo" will move the objects at
  "/foo/box1" and "/foo/robots/HAL9000".
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path "Meshcat paths" for the semantics.
  @param X_ParentPath the relative transform from the path to its immediate
  parent.

  @pydrake_mkdoc_identifier{RigidTransform}
  */
  void SetTransform(std::string_view path,
                    const math::RigidTransformd& X_ParentPath);

  /** Set the homogeneous transform for a given path in the scene tree relative
  to its parent path. An object's pose is the concatenation of all of the
  transforms along its path, so setting the transform of "/foo" will move the
  objects at "/foo/box1" and "/foo/robots/HAL9000".
  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param matrix the relative transform from the path to its immediate
                parent.

  Note: Prefer to use the overload which takes a RigidTransformd unless you need
  the fully parameterized homogeneous transform (which additionally allows
  scale and sheer).

  @pydrake_mkdoc_identifier{matrix}
  */
  void SetTransform(std::string_view path,
                    const Eigen::Ref<const Eigen::Matrix4d>& matrix);

  /** Deletes the object at the given `path` as well as all of its children.
  See @ref meshcat_path for the detailed semantics of deletion. */
  void Delete(std::string_view path = "");

  // TODO(#16486): add low-pass filter to smooth the Realtime plot
  /** Sets the realtime rate that is displayed in the meshcat visualizer stats
   strip chart. This rate is the ratio between sim time and real
   world time. 1 indicates the simulator is the same speed as real time. 2
   indicates running twice as fast as real time, 0.5 is half speed, etc.
  @see drake::systems::Simulator::set_target_realtime_rate()
  @param rate the realtime rate value to be displayed, will be converted to a
  percentage (multiplied by 100) */
  void SetRealtimeRate(double rate);

  /** Sets a single named property of the object at the given path. For example,
  @verbatim
  meshcat.SetProperty("/Background", "visible", false);
  @endverbatim
  will turn off the background. See @ref meshcat_path "Meshcat paths" for more
  details about these properties and how to address them.

  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.

  @pydrake_mkdoc_identifier{bool}
  */
  void SetProperty(std::string_view path, std::string property, bool value);

  /** Sets a single named property of the object at the given path. For example,
  @verbatim
  meshcat.SetProperty("/Lights/DirectionalLight/<object>", "intensity", 1.0);
  @endverbatim
  See @ref meshcat_path "Meshcat paths" for more details about these properties
  and how to address them.

  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.

  @pydrake_mkdoc_identifier{double}
  */
  void SetProperty(std::string_view path, std::string property, double value);

  /** Sets a single named property of the object at the given path. For example,
  @verbatim
  meshcat.SetProperty("/Background", "top_color", {1.0, 0.0, 0.0});
  @endverbatim
  See @ref meshcat_path "Meshcat paths" for more details about these properties
  and how to address them.

  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.

  @pydrake_mkdoc_identifier{vector_double}
  */
  void SetProperty(std::string_view path, std::string property,
                   const std::vector<double>& value);

  // TODO(russt): Support multiple animations, by name.  Currently "default" is
  // hard-coded in the meshcat javascript.
  /** Sets the MeshcatAnimation, which creates a slider interface element to
  play/pause/rewind through a series of animation frames in the visualizer. */
  void SetAnimation(const MeshcatAnimation& animation);

  /** @name Meshcat Controls
   Meshcat "Controls" are user interface elements in the browser.  These
   currently include buttons and sliders.
   - Controls appear in the browser under the "Open Controls" dropdown menu,
     and are appended to the root node of the tree (they do not have a path in
     the "scene tree"), in the order in which they are added.
   - Controls must have unique names.  Adding a control with a name that was
     already in use will overwrite the existing control.  If you add a slider
     with a name that was previous used for a button, then the button will be
     replaced by the slider, etc.
   - Users of this Meshcat instance are responsible for polling the GUI to
     retrieve the user interface events.
   - Controls can be added/deleted/accessed anytime during the lifetime of this
     Meshcat instance.
  */
  //@{

  /** Adds a button with the label `name` to the meshcat browser controls GUI.
   If the button already existed, then resets its click count to zero instead.
   @throws std::exception if `name` has already been added as any other type
   of control (e.g., slider). */
  void AddButton(std::string name);

  /** Returns the number of times the button `name` has been clicked in the
   GUI, from the time that it was added to `this`.  If multiple browsers are
   open, then this number is the cumulative number of clicks in all browsers.
   @throws std::exception if `name` is not a registered button. */
  int GetButtonClicks(std::string_view name);

  /** Removes the button `name` from the GUI.
   @throws std::exception if `name` is not a registered button. */
  void DeleteButton(std::string name);

  /** Adds a slider with the label `name` to the meshcat browser controls GUI.
   The slider range is given by [`min`, `max`]. `step` is the smallest
   increment by which the slider can change values (and therefore send updates
   back to this Meshcat instance). `value` specifies the initial value; it will
   be truncated to the slider range and rounded to the nearest increment.
   @throws std::exception if `name` has already been added as any type of
   control (e.g., either button or slider). */
  void AddSlider(std::string name, double min, double max, double step,
                 double value);

  /** Sets the current `value` of the slider `name`. `value` will be truncated
   to the slider range and rounded to the nearest increment specified by the
   slider `step`. This will update the slider element in all connected meshcat
   browsers.
   @throws std::exception if `name` is not a registered slider. */
  void SetSliderValue(std::string name, double value);

  /** Gets the current `value` of the slider `name`.
   @throws std::exception if `name` is not a registered slider. */
  double GetSliderValue(std::string_view name);

  /** Removes the slider `name` from the GUI.
   @throws std::exception if `name` is not a registered slider. */
  void DeleteSlider(std::string name);

  /** Removes all buttons and sliders from the GUI that have been registered by
   this Meshcat instance. It does *not* clear the default GUI elements set in
   the meshcat browser (e.g. for cameras and lights). */
  void DeleteAddedControls();

  //@}

  /** Returns an HTML string that can be saved to a file for a snapshot of the
  visualizer and its contents. The HTML can be viewed in the browser
  without any connection to a Meshcat "server" (e.g. `this`). This is a great
  way to save and share your 3D content.

  Note that controls (e.g. sliders and buttons) are not included in the HTML
  output, because their usefulness relies on a connection to the server. */
  std::string StaticHtml();

  /* These remaining public methods are intended to primarily for testing. These
  calls must safely acquire the data from the websocket thread and will block
  execution waiting for that data to be acquired. They are intentionally
  excluded from doxygen.

  The SceneTree has a directory-like structure.  If HasPath() is true, then that
  path optionally has an object, a transform, and any number of properties.
  */

  /* Returns true iff `path` exists in the current internal (server) tree.
  Note that the javascript client maintains its own tree, which may contain
  elements that are not in the server tree. */
  bool HasPath(std::string_view path) const;

  /* Returns the msgpack representation of the object stored in the internal
  (server) tree at `path`, or the empty string if `path` does not exist or no
  object has been set at `path`. */
  std::string GetPackedObject(std::string_view path) const;

  /* Returns the msgpack representation of the transform stored in the internal
  (server) tree at `path`, or the empty string if `path` does not exist or no
  transform has been set at `path`. */
  std::string GetPackedTransform(std::string_view path) const;

  /* Returns the msgpack representation of the `property` stored in the
  internal (server) tree at `path`, or the empty string if `path` does not
  exist or `property` has not been set at `path`. */
  std::string GetPackedProperty(std::string_view path,
                                std::string property) const;

#ifndef DRAKE_DOXYGEN_CXX
  /* (Internal use for unit testing only) Causes the websocket worker thread to
  exit with an error, which will spit out an exception from the next Meshcat
  main thread function that gets called. The fault_number selects which fault to
  inject, between 0 and kMaxFaultNumber inclusive; refer to the implementation
  for details on meaning of each number. */
  void InjectWebsocketThreadFault(int fault_number);

  /* (Internal use for unit testing only) The max value (inclusive) for
  fault_number, above. */
  static constexpr int kMaxFaultNumber = 3;
#endif

 private:
  // Provides PIMPL encapsulation of websocket types.
  class Impl;

  // Safe accessors for the PIMPL object.
  Impl& impl();
  const Impl& impl() const;

  // Always a non-nullptr Impl, but stored as void* to enforce that the
  // impl() accessors are always used.
  void* const impl_{};
};

}  // namespace geometry
}  // namespace drake
