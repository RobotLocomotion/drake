#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

/** Provides an interface to %Meshcat (https://github.com/rdeits/meshcat).

Each instance of this class spawns a thread which runs an http/websocket server.
Users can navigate their browser to the hosted URL to visualize the Meshcat
scene.  Note that, unlike many visualizers, one cannot open the visualizer until
this server is running.

In the current implementation, Meshcat methods must be called from the same
thread where the class instance was constructed.  For example, running multiple
simulations in parallel using the same Meshcat instance is not yet supported. We
may generalize this in the future.

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

  /** Constructs the Meshcat instance. It will listen on the first available
  port starting at 7001 (up to 7099). */
  Meshcat();

  ~Meshcat();

  /** Returns the hosted http URL. */
  std::string web_url() const;

  /** (Advanced) Returns the ws:// URL for direct connection to the websocket
  interface.  Most users should connect via a browser opened to web_url(). */
  std::string ws_url() const;

  /** Sets the 3D object at a given `path` in the scene tree.  Note that
  `path`="/foo" will always set an object in the tree at "/foo/<object>".  See
  @ref meshcat_path.  Any objects previously set at this `path` will be
  replaced.
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path "Meshcat paths" for the semantics.
  @param shape a Shape that specifies the geometry of the object.
  @param rgba an Rgba that specifies the (solid) color of the object.
  */
  void SetObject(std::string_view path, const Shape& shape,
                 const Rgba& rgba = Rgba(.9, .9, .9, 1.));

  // TODO(russt): SetObject with texture map.

  /** Set the RigidTransform for a given path in the scene tree. An object's
  pose is the concatenation of all of the transforms along its path, so setting
  the transform of "/foo" will move the objects at "/foo/box1" and
  "/foo/robots/HAL9000".
  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path "Meshcat paths" for the semantics.
  @param X_ParentPath the relative transform from the path to its immediate
  parent.
  */
  void SetTransform(std::string_view path,
                    const math::RigidTransformd& X_ParentPath);

  /** Deletes the object at the given `path` as well as all of its children.
  See @ref meshcat_path for the detailed semantics of deletion. */
  void Delete(std::string_view path = "");

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
  */
  void SetProperty(std::string_view path, std::string property, bool value);

  /** Sets a single named property of the object at the given path. For example,
  @verbatim
  meshcat.SetProperty("/Cameras/default/rotated/<object>", "zoom", 2.0);
  meshcat.SetProperty("/Lights/DirectionalLight/<object>", "intensity", 1.0);
  @endverbatim
  See @ref meshcat_path "Meshcat paths" for more details about these properties
  and how to address them.

  @param path a "/"-delimited string indicating the path in the scene tree.
              See @ref meshcat_path for the semantics.
  @param property the string name of the property to set
  @param value the new value.
  */
  void SetProperty(std::string_view path, std::string property, double value);

  // TODO(russt): Implement SetAnimation().
  // TODO(russt): Implement SetButton() and SetSlider() as wrappers on
  // set_control.

  /* These remaining public methods are intended to primarily for testing. These
  calls must safely acquire the data from the websocket thread and will block
  execution waiting for that data to be acquired. They are intentionally
  excluded from doxygen. */

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

 private:
  // Provides PIMPL encapsulation of websocket types.
  class WebSocketPublisher;
  std::unique_ptr<WebSocketPublisher> publisher_;
};

}  // namespace geometry
}  // namespace drake
