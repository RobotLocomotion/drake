#pragma once

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/name_value.h"
#include "drake/common/timer.h"
#include "drake/geometry/meshcat_animation.h"
#include "drake/geometry/meshcat_params.h"
#include "drake/geometry/proximity/triangle_surface_mesh.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"
// TODO(russt): Move point_cloud.h to a more central location.
#include "drake/perception/point_cloud.h"

namespace drake {
namespace geometry {

// Forward-declare a helper class from meshcat_recording_internal.h.
#ifndef DRAKE_DOXYGEN_CXX
namespace internal {
class MeshcatRecording;
}  // namespace internal
#endif

/** Provides an interface to %Meshcat (https://github.com/meshcat-dev/meshcat).

Each instance of this class spawns a thread which runs an http/websocket server.
Users can navigate their browser to the hosted URL to visualize the Meshcat
scene.  Note that, unlike many visualizers, one cannot open the visualizer until
this server is running.

@warning In the current implementation, Meshcat methods must be called from the
same thread where the class instance was constructed.  For example, running
multiple simulations in parallel using the same Meshcat instance is not yet
supported. We may generalize this in the future.

@section meshcat_path Meshcat paths and the scene tree

https://github.com/meshcat-dev/meshcat#api provides a nice introduction to the
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
https://github.com/meshcat-dev/meshcat#api meshcat documentation @endlink and
the @link https://threejs.org/docs/index.html three.js documentation @endlink.
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

@section meshcat_url_parameters Parameters for the hosted Meshcat page

%Meshcat has an *experimental* AR/VR option (using WebXR). It can be enabled
through url parameters. For example, for a meshcat url `http://localhost:7000`,
the following will enable the VR mode:

    http://localhost:7000?webxr=vr

To to use augmented reality (where the meshcat background is replaced with your
device's camera image), use:

    http://localhost:7000?webxr=ar

If augmented reality is not available, it will fallback to VR mode.

Some notes on using the AR/VR modes:

  - Before starting the WebXR session, position the interactive camera to be
    approximately where you want the origin of the head set's origin to be.
  - The meshcat scene controls are disabled while the WebXR session is active.
  - WebXR sessions can only be run with *perspective* cameras.
  - The controllers can be *visualized* but currently can't interact with the
    Drake simulation physically. To visualize the controllers append the
    additional url parameter `controller=on` as in
    `http://localhost:7000?webxr=vr&controller=on`.

If you do not have AR/VR hardware, you can use an emulator in your browser to
experiment with the mode. Use an browser plugin like WebXR API Emulator (i.e.,
for
[Chrome](https://chrome.google.com/webstore/detail/webxr-api-emulator/mjddjgeghkdijejnciaefnkjmkafnnje)
or
[Firefox](https://addons.mozilla.org/en-US/firefox/addon/webxr-api-emulator/)).

The AR/VR mode is not currently supported in offline mode (i.e., when saving as
StaticHtml()).

@section network_access Network access

See MeshcatParams for options to control the hostname and port to bind to.

See \ref allow_network "DRAKE_ALLOW_NETWORK" for an environment variable
option to deny %Meshcat entirely. */
class Meshcat {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Meshcat);

  // Defines which side of faces will be rendered - front, back or both.
  // From https://github.com/mrdoob/three.js/blob/dev/src/constants.js
  enum SideOfFaceToRender { kFrontSide = 0, kBackSide = 1, kDoubleSide = 2 };

  /** Constructs the %Meshcat instance on `port`. If no port is specified,
  it will listen on the first available port starting at 7000 (up to 7999).
  If port 0 is specified, it will listen on an arbitrary "ephemeral" port.
  @pre We require `port` == 0 || `port` >= 1024.
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
  @note If `shape` is a mesh, the file referred to can be either an .obj file
  or an _embedded_ .gltf file (it has all geometry data and texture data
  contained within the single .gltf file).
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
                 bool wireframe = false, double wireframe_line_width = 1.0,
                 SideOfFaceToRender side = kDoubleSide);

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

  /** Sets the "object" at `path` in the scene tree to a triangular mesh.

  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param vertices is a 3-by-N matrix of 3D point defining the vertices of the
                  mesh.
  @param faces is a 3-by-M integer matrix with each entry denoting an index
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
                       double wireframe_line_width = 1.0,
                       SideOfFaceToRender side = kDoubleSide);

  /** Sets the "object" at `path` in the scene tree to a triangular mesh with
  per-vertex coloring.

  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param vertices is a 3-by-N matrix of 3D point defining the vertices of the
                  mesh.
  @param faces is a 3-by-M integer matrix with each entry denoting an index
               into vertices and each column denoting one face (aka
               SurfaceTriangle).
  @param colors is a 3-by-N matrix of RGB color values, one color per vertex of
                the mesh. Color values are in the range [0, 1].
  @param wireframe if "true", then only the triangle edges are visualized, not
                   the faces.
  @param wireframe_line_width is the width in pixels.  Due to limitations in
                              WebGL implementations, the line width may be 1
                              regardless of the set value. */
  void SetTriangleColorMesh(std::string_view path,
                            const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                            const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                            const Eigen::Ref<const Eigen::Matrix3Xd>& colors,
                            bool wireframe = false,
                            double wireframe_line_width = 1.0,
                            SideOfFaceToRender side = kDoubleSide);

  // TODO(russt): Add support for per vertex colors / colormaps.
  /** Sets the "object" at `path` to be a triangle surface mesh representing a
  3D surface, via an API that roughly follows matplotlib's plot_surface()
  method.

  @param X matrix of `x` coordinate values defining the vertices of the mesh.
  @param Y matrix of `y` coordinate values.
  @param Z matrix of `z` coordinate values.
  @param rgba is the mesh face or wireframe color.
  @param wireframe if "true", then only the triangle edges are visualized, not
                   the faces.
  @param wireframe_line_width is the width in pixels.  Due to limitations in
                              WebGL implementations, the line width may be 1
                              regardless of the set value.

  Typically, X and Y are obtained via, e.g.
  @code
  constexpr int nx = 15, ny = 11;
  X = RowVector<double, nx>::LinSpaced(0, 1).replicate<ny, 1>();
  Y = Vector<double, ny>::LinSpaced(0, 1).replicate<1, nx>();
  @endcode
  in C++ or e.g.
  @code
  xs = np.linspace(0, 1, 15)
  ys = np.linspace(0, 1, 11)
  [X, Y] = np.meshgrid(xs, ys)
  @endcode
  in Python, and Z is the surface evaluated on each X, Y value.

  @pre X, Y, and Z must be the same shape. */
  void PlotSurface(std::string_view path,
                   const Eigen::Ref<const Eigen::MatrixXd>& X,
                   const Eigen::Ref<const Eigen::MatrixXd>& Y,
                   const Eigen::Ref<const Eigen::MatrixXd>& Z,
                   const Rgba& rgba = Rgba(0.1, 0.1, 0.9, 1.0),
                   bool wireframe = false, double wireframe_line_width = 1.0);

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

  /** Resets the default camera, camera target, background, grid lines, and axes
   to their default settings. */
  void ResetRenderMode();

  // TODO(SeanCurtis-TRI): Once meshcat supports animation of camera target,
  // add the ability to animate this and SetCameraPose().

  /** Positions the camera's view target point T to the location in
   `target_in_world` (`p_WT`).

   If the camera is orthographic (i.e., by calling Set2DRenderMode() or
   SetCamera(OrthographicCamera)), this will have no effect.

   @warning Setting the target position to be coincident with the camera
   position will lead to undefined behavior.

   @param target_in_world the position of the target point T in Drake's z-up
               world frame (p_WT). */
  void SetCameraTarget(const Eigen::Vector3d& target_in_world);

  /** A convenience function for positioning the camera and its view target
   in the world frame. The camera is placed at `camera_in_world` and looks
   toward `target_in_world`. The camera is oriented around this view direction
   so that the camera's up vector points in the positive Wz direction as much
   as possible.

   Unlike SetCameraTarget() this can be used to orient orthographic cameras
   as well.

   @note This is Drake's z-up world frame and not the three.js world frame
   you'd have to use if you set the "position" on the camera directly.

   @warning The behavior is undefined if camera and target positions are
   coincident.

   @param camera_in_world the position of the camera's origin C in Drake's z-up
               world frame (p_WC).
   @param target_in_world the position of the target point T in Drake's z-up
               world frame (p_WT). */
  void SetCameraPose(const Eigen::Vector3d& camera_in_world,
                     const Eigen::Vector3d& target_in_world);

  /** Returns the most recently received camera pose.

   A meshcat browser session can be configured to transmit its camera pose.
   It is enabled by appending a url parameter. For example, if the url for the
   meshcat server is:

       http://localhost:7000

   A particular browser can be configured to transmit its camera pose back to
   Drake by supplying the following url:

       http://localhost:7000/?tracked_camera=on

   It is possible to use that URL in multiple browsers simultaneously. A
   particular view will only transmit its camera position when its camera
   position actually *changes*. As such, the returned camera pose will reflect
   the pose of the camera from that most-recently manipulated browser.

   std::nullopt is returned if:

     - No meshcat session has transmitted its camera pose.
     - The meshcat session that last transmitted its pose is no longer
       connected.
     - The meshcat session transmitting has an orthographic camera.

  <!-- Note to developer. This logic is tested in the python test
   meshcat_camera_tracking_test.py. -->
   */
  std::optional<math::RigidTransformd> GetTrackedCameraPose() const;

  // TODO(SeanCurtis-TRI): Consider the API:
  //  void SetCameraPose(const RigidTransformd& X_WC, bool target_distance = 1);
  // We'll have to confirm that picking arbitrary rotations R_WC doesn't
  // fight badly with the camera controller.

  /** Set the RigidTransform for a given path in the scene tree relative to its
  parent path. An object's pose is the concatenation of all of the transforms
  along its path, so setting the transform of "/foo" will move the objects at
  "/foo/box1" and "/foo/robots/HAL9000".
  @param path a "/"-delimited string indicating the path in the scene tree. See
              @ref meshcat_path "Meshcat paths" for the semantics.
  @param X_ParentPath the relative transform from the path to its immediate
              parent.
  @param time_in_recording (optional). If recording (see StartRecording()), then
              in addition to publishing the transform to any meshcat browsers
              immediately, this transform is saved to the current animation at
              `time_in_recording`.

  @pydrake_mkdoc_identifier{RigidTransform}
  */
  void SetTransform(std::string_view path,
                    const math::RigidTransformd& X_ParentPath,
                    std::optional<double> time_in_recording = std::nullopt);

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

  Note: Meshcat does not properly support non-uniform scaling. See Drake issue
  #18095.

  @pydrake_mkdoc_identifier{matrix}
  */
  void SetTransform(std::string_view path,
                    const Eigen::Ref<const Eigen::Matrix4d>& matrix);

  /** Deletes the object at the given `path` as well as all of its children.
  See @ref meshcat_path for the detailed semantics of deletion. */
  void Delete(std::string_view path = "");

  /** @name Realtime Rate Reporting

   %Meshcat can be used to visualize the realtime rate of a simulation's
   computation in the meshcat visualizer webpage. Meshcat broadcasts a realtime
   rate message that is received and displayed by the browser in a
   <a href="https://github.com/mrdoob/stats.js">stats strip chart</a>

   Advancing the simulation's time requires a certain amount of real world
   compute time. The realtime rate, a non-negative real value, is the ratio of
   the sim time to real world time. A value of one indicates that the simulation
   is advancing at the same rate as the real world clock. Higher values indicate
   faster simulations. Lower values indicate slower simulations.

   The realtime rate value can be broadcast to the visualizer in one of two
   ways:

     - Throttled - Meshcat is configured to broadcast realtime rate data
       in a "smoothed" manner, so that the visualization doesn't fluctuate at
       arbitrarily high rates and possibly make the value difficult to read.
       See SetSimulationTime(). This is the preferred means of reporting real
       time rate.
     - Immediate - a %Meshcat user _can_ compute realtime rate themselves and
       simply ask %Meshcat to dispatch a message with that computed value.
       See SetRealtimeRate(). This is not recommended and the user is
       responsible for computing the value and controlling the frequency at
       which this is done.
   */
  //@{

  /** Updates %Meshcat's knowledge of simulation time. Changes to simulation
   time *may* trigger a realtime rate message to the meshcat visualizer client
   based on the configured value in MeshcatParams::realtime_rate_period value.

   Invoking this method _may_ dispatch a message to clients. The following rules
   apply to invocations and messages:

     - The first invocation is necessary to _initialize_ the calculation; it
       defines the the starting point from which all calculations are performed
       (for both wall clock time as well as simulation time). As no interval can
       be measured from a single invocation, no rate can be computed. Therefore,
       the first invocation will *never* broadcast the message.
     - Wall clock time must advance at least MeshcatParams::realtime_rate_period
       seconds for a message to be sent.
     - Meshcat promises to broadcast one message per elapsed period -- starting
       from initialization -- regardless of the frequency at which
       SetSimulationTime() actually gets invoked. If the elapsed time between
       invocations exceeds MeshcatParams::realtime_rate_period, multiple
       messages will be broadcast; one for each complete period.
       - This implies that each column of pixels in the realtime rate chart in
         the client visualizer represents a fixed amount of wall clock time.

   When the realtime rate is broadcast, that value will be reported by
   GetRealtimeRate().

   The realtime rate calculation can be "reset" by passing a simulation time
   value that is *strictly less* than the value of the previous invocation. This
   has the effect of re-initializing the calculation with the passed time and
   the wall clock time of the invocation. So, if the simulation's context gets
   reset to `time = 0`, calls to this method passing the context time will
   implicitly reset realtime rate calculations accordingly. Resetting the
   calculator will reset the value reported by GetRealtimeRate() to zero.

   This function is safe to be called redundantly with the same simulation time.
   Redundant calls are a no-op.

   @param sim_time  The *absolute* sim time being visualized.

   @see drake::systems::Simulator::set_target_realtime_rate() */
  void SetSimulationTime(double sim_time);

  /** Gets the last time value passed to SetSimulationTime(). */
  double GetSimulationTime() const;

  /** Immediately broadcasts the given realtime rate to all connected clients.
  @param rate the realtime rate value to broadcast, will be converted to a
  percentage (multiplied by 100) */
  void SetRealtimeRate(double rate);

  /** Gets the realtime rate that was last broadcast by this instance
   (typically, the value displayed in the meshcat visualizer stats chart).
   See SetRealtimeRate(). */
  double GetRealtimeRate() const;

  //@}

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
  @param time_in_recording (optional). If recording (see StartRecording()), then
              in addition to publishing the property to any meshcat browsers
              immediately, this transform is saved to the current animation at
              `time_in_recording`.

  @pydrake_mkdoc_identifier{bool}
  */
  void SetProperty(std::string_view path, std::string property, bool value,
                   std::optional<double> time_in_recording = std::nullopt);

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
  @param time_in_recording (optional) the time at which this property should
              be applied, if Meshcat is current recording (see
              StartRecording()). If Meshcat is not currently recording, then
              this value is simply ignored.

  @pydrake_mkdoc_identifier{double}
  */
  void SetProperty(std::string_view path, std::string property, double value,
                   std::optional<double> time_in_recording = std::nullopt);

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
  @param time_in_recording (optional) the time at which this property should
              be applied, if Meshcat is current recording (see
              StartRecording()). If Meshcat is not currently recording, then
              this value is simply ignored.

  @pydrake_mkdoc_identifier{vector_double}
  */
  void SetProperty(std::string_view path, std::string property,
                   const std::vector<double>& value,
                   std::optional<double> time_in_recording = std::nullopt);

  /** Sets the *environment* texture. For objects with physically-based
   rendering (PBR) material properties (e.g., metallic surfaces), this defines
   the luminance environment, contributing to total illumination and appearing
   in reflections.

   The image should be of a format typically supported by web browsers: e.g.,
   jpg, png, etc. Furthermore, the image must be an
   <a href="https://en.wikipedia.org/wiki/Equirectangular_projection">
   equirectangular image</a> (as opposed to a
   <a href="https://en.wikipedia.org/wiki/Cube_mapping">cube-map</a>).

   If the path is empty, the environment map will be cleared.

   @throws if `image_path` is *not* empty and the file isn't accessible.
   @pre If `image_path` names an accessible file, it is an appropriate image
        type. */
  void SetEnvironmentMap(const std::filesystem::path& image_path);

  // TODO(russt): Support multiple animations, by name.  Currently "default" is
  // hard-coded in the meshcat javascript.
  /** Sets the MeshcatAnimation, which creates a slider interface element to
  play/pause/rewind through a series of animation frames in the visualizer.

  See also StartRecording(), which records supported calls to `this` into a
  MeshcatAnimation, and PublishRecording(), which calls SetAnimation() with the
  recording. */
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
   If the optional @p keycode is set to a javascript string key code (such as
   "KeyG" or "ArrowLeft", see
   https://developer.mozilla.org/en-US/docs/Web/API/UI_Events/Keyboard_event_code_values),
   then a keydown callback is registered in the GUI which will also `click` the
   button. If the button already existed, then resets its click count to zero;
   and sets the keycode if no keycode was set before.

   @throws std::exception if `name` has already been added as any other type of
   control (e.g., slider).
   @throw std::exception if a button of the same `name` has already been
   assigned a different keycode. */
  void AddButton(std::string name, std::string keycode = "");

  /** Returns the number of times the button `name` has been clicked in the
   GUI, from the time that it was added to `this`.  If multiple browsers are
   open, then this number is the cumulative number of clicks in all browsers.
   @throws std::exception if `name` is not a registered button. */
  int GetButtonClicks(std::string_view name) const;

  /** Returns the names of all buttons. */
  std::vector<std::string> GetButtonNames() const;

  /** Removes the button `name` from the GUI.
   @returns true iff the button was removed.
   @throws std::exception if `strict` is true and `name` is not a registered
           button. */
  bool DeleteButton(std::string name, bool strict = true);

  /** Adds a slider with the label `name` to the meshcat browser controls GUI.
   The slider range is given by [`min`, `max`]. `step` is the smallest
   increment by which the slider can change values (and therefore send updates
   back to this Meshcat instance). `value` specifies the initial value; it will
   be truncated to the slider range and rounded to the nearest increment. If
   the optional @p decrement_keycode or @p increment_keycode are set to a
   javascript string key code (such as "KeyG" or "ArrowLeft", see
   https://developer.mozilla.org/en-US/docs/Web/API/UI_Events/Keyboard_event_code_values),
   then keydown callbacks will be registered in the GUI that will move the
   slider by @p step (within the limits) when those buttons are pressed.

   @returns the truncated and rounded value that was actually set.
   @throws std::exception if `name` has already been added as any type of
   control (e.g., either button or slider). */
  double AddSlider(std::string name, double min, double max, double step,
                   double value, std::string decrement_keycode = "",
                   std::string increment_keycode = "");

  /** Sets the current `value` of the slider `name`. `value` will be truncated
   to the slider range and rounded to the nearest increment specified by the
   slider `step`. This will update the slider element in all connected meshcat
   browsers.
   @returns the truncated and rounded value that was actually set.
   @throws std::exception if `name` is not a registered slider. */
  double SetSliderValue(std::string name, double value);

  /** Gets the current `value` of the slider `name`.
   @throws std::exception if `name` is not a registered slider. */
  double GetSliderValue(std::string_view name) const;

  /** Returns the names of all sliders. */
  std::vector<std::string> GetSliderNames() const;

  /** Removes the slider `name` from the GUI.
   @returns true iff the slider was removed.
   @throws std::exception if `strict` is true and `name` is not a registered
           slider. */
  bool DeleteSlider(std::string name, bool strict = true);

  /** Removes all buttons and sliders from the GUI that have been registered by
   this Meshcat instance. It does *not* clear the default GUI elements set in
   the meshcat browser (e.g. for cameras and lights). */
  void DeleteAddedControls();

  /** Status of a gamepad obtained from the Meshcat javascript client. */
  struct Gamepad {
    /** Passes this object to an Archive.
    Refer to @ref yaml_serialization "YAML Serialization" for background. */
    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(DRAKE_NVP(index));
      a->Visit(DRAKE_NVP(button_values));
      a->Visit(DRAKE_NVP(axes));
    }

    // Descriptions of these properties were adapted from
    // https://developer.mozilla.org/en-US/docs/Web/API/Gamepad.

    /** An an integer that is auto-incremented to be unique for each device
    currently connected to the system. If `index.has_value() == false`, then we
    have not yet received any gamepad status from the Meshcat browser. */
    std::optional<int> index;

    /** An array of floating point values representing analog buttons, such as
    the triggers on many modern gamepads. The values are normalized to the
    range [0.0, 1.0], with 0.0 representing a button that is not pressed, and
    1.0 representing a button that is fully pressed.

    See https://w3c.github.io/gamepad/#dfn-standard-gamepad for the standard
    mapping of gamepad buttons to this vector.
    */
    std::vector<double> button_values;

    /** An array of floating point values representing e.g. analog thumbsticks.
    Each entry in the array is a floating point value in the range -1.0 – 1.0,
    representing the axis position from the lowest value (-1.0) to the highest
    value (1.0).

    In the standard gamepad mapping, we have:
    - axes[0] Left stick x (negative left/positive right)
    - axes[1] Left stick y (negative up/positive down)
    - axes[2] Right stick x (negative left/positive right)
    - axes[3] Right stick y (negative up/positive down)

    Note that a stick that is left alone may not output all zeros.
    https://beej.us/blog/data/javascript-gamepad/ gives some useful advice for
    applying a deadzone to these values.
    */
    std::vector<double> axes;
  };

  /** Returns the status from the most recently updated gamepad data in the
  Meshcat. See Gamepad for details on the returned values.

  Note that in javascript, gamepads are not detected until users "opt-in" by
  pressing a gamepad button or moving the thumbstick in the Meshcat window. If
  no gamepad information is available in javascript, then no messages are sent
  and the returned gamepad index will not have a value.

  Currently Meshcat only attempts to support one gamepad. If multiple gamepads
  are detected in the same Meshcat window, then only the status of the first
  connected gamepad in `navigator.GetGamepads()` is returned. If multiple
  Meshcat windows are connected to this Meshcat instance, and gamepads are
  being used in multiple windows, then the returned status will be the most
  recently received status message. Therefore using multiple gamepads
  simultaneously is not recommended.

  This feature is provided primarily to support applications where Drake is
  running on a remote machine (e.g. in the cloud), and the Meshcat javascript
  in the browser is the only code running on the local machine which has access
  to the gamepad.

  For more details on javascript support for gamepads (or to test that your
  gamepad is working), see https://beej.us/blog/data/javascript-gamepad/. */
  Gamepad GetGamepad() const;

  //@}

  /** Returns an HTML string that can be saved to a file for a snapshot of the
  visualizer and its contents. The HTML can be viewed in the browser
  without any connection to a Meshcat "server" (e.g. `this`). This is a great
  way to save and share your 3D content.

  Note that controls (e.g. sliders and buttons) are not included in the HTML
  output, because their usefulness relies on a connection to the server.

  You can also use your browser to download this file, by typing "/download"
  on the end of the URL (i.e., accessing `web_url() + "/download"`). */
  std::string StaticHtml() const;

  /** Like StaticHtml(), returns a standalone snapshot of the visualizer and its
  contents; the return value is a ZIP file containing a thin `meshcat.html` page
  and the assets (meshes, textures, etc.) as separate files.

  When you are uploading the unzipped files to a website, this will typically be
  a more efficient representation as compared to StaticHtml(). However, it
  cannot be opened directly by a browser from disk. A simple web server like
  `python -m http.server` is required.

  You can also use your browser to download this file, by typing "/download.zip"
  on the end of the URL (i.e., accessing `web_url() + "/download.zip"`). */
  std::string StaticZip() const;

  /** Sets a flag indicating that subsequent calls to SetTransform and
  SetProperty should also be "recorded" into a MeshcatAnimation when their
  optional time_in_recording argument is supplied.  The data in these events
  will be combined with any frames previously added to the animation; if the
  same transform/property is set at the same time, then it will overwrite the
  existing frame in the animation.

  @param set_visualizations_while_recording if true, then each method will send
  the visualization immediately to Meshcat *and* record the visualization in
  the animation. Set to false to avoid updating the visualization during
  recording. One exception is calls to SetObject, which will always be sent to
  the visualizer immediately (because meshcat animations do not support
  SetObject).
  */
  void StartRecording(double frames_per_second = 64.0,
                      bool set_visualizations_while_recording = true);

  /** Sets a flag to pause/stop recording.  When stopped, publish events will
  not add frames to the animation. */
  void StopRecording();

  /** Sends the recording to Meshcat as an animation. The published animation
  only includes transforms and properties; the objects that they modify must be
  sent to the visualizer separately (e.g. by calling Publish()). */
  void PublishRecording();

  /** Deletes the current animation holding the recorded frames.  Animation
  options (autoplay, repetitions, etc) will also be reset, and any pointers
  obtained from get_mutable_recording() will be rendered invalid. This does
  *not* currently remove the animation from Meshcat. */
  void DeleteRecording();

  /** Returns a const reference to this Meshcat's MeshcatAnimation object. This
  can be used to check animation properties (e.g., autoplay). The return value
  will only remain valid for the lifetime of `this` or until DeleteRecording()
  is called. */
  const MeshcatAnimation& get_recording() const;

  /** Returns a mutable reference to this Meshcat's MeshcatAnimation object.
  This can be used to set animation properties (like autoplay, the loop mode,
  number of repetitions, etc). The return value will only remain valid for the
  lifetime of `this` or until DeleteRecording() is called. */
  MeshcatAnimation& get_mutable_recording();

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
  /* (Internal use for unit testing only) Injects a websocket message as if it
  came from a web browser. Note that this skips the entire network stack, so the
  `ws` pointer will be null during message handling; some messages (e.g., slider
  controls) do not allow a null pointer. This function blocks until the message
  has been handled. Search meshcat_test for uses of `msgpack::pack` for examples
  of how to prepare the `message` bytes. */
  void InjectWebsocketMessage(std::string_view message);

  /* (Internal use for unit testing only) Causes the websocket worker thread to
  exit with an error, which will spit out an exception from the next Meshcat
  main thread function that gets called. The fault_number selects which fault to
  inject, between 0 and kMaxFaultNumber inclusive; refer to the implementation
  for details on meaning of each number. */
  void InjectWebsocketThreadFault(int fault_number);

  /* (Internal use for unit testing only) The max value (inclusive) for
  fault_number, above. */
  static constexpr int kMaxFaultNumber = 3;

  /* (Internal use for unit testing only) Used to mock the monotonic wall time
   source to control time during unit testing.  */
  void InjectMockTimer(std::unique_ptr<Timer>);
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

  /* Encapsulated recording logic. The value is never nullptr. */
  std::unique_ptr<internal::MeshcatRecording> recording_;
};

}  // namespace geometry
}  // namespace drake
