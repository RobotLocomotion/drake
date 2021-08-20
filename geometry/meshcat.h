#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"
#include "drake/geometry/rgba.h"
#include "drake/geometry/shape_specification.h"
#include "drake/math/rigid_transform.h"

namespace drake {
namespace geometry {

/** Provides an interface to Meshcat (https://github.com/rdeits/meshcat).

Each instance of this class spawns a thread which runs an http/websocket server.
Users can navigate their browser to the hosted URL to visualize the Meshcat
scene.  Note that, unlike many visualizers, one cannot open the visualizer until
this server is running.
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

  /** Returns the prefix that gets added to any path that does not begin with
  '/'. The default value is "drake". */
  const std::string& prefix() const;

  /** Sets the prefix that gets added to any path that does not begin with '/'.
  Any leading or trailing '/' on `prefix` will be removed. */
  void set_prefix(std::string_view prefix);

  /** Sets the 3D object at a given path in the scene tree.
  @param path a "/"-separated string indicating the path in the scene tree.
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
  @param path a "/"-separated string indicating the path in the scene tree.
  @param X_PathParent the relative transform from the path to it's immediate
  parent.
  */
  void SetTransform(std::string_view path,
                    const math::RigidTransformd& X_PathParent);

  /** Deletes the object at the given `path` as well as all of its children.
  Deleting the empty string "" is the default, this deletes the prefix().
  Deleting a `path` that does not exist in the current tree does nothing.
  Deleting the root node "/" does nothing.

  @param path a "/"-separated string indicating the path in the scene tree. */
  void Delete(std::string_view path = "");

  /** Sets a single named property of the object at the given path. For example,
  @verbatim
  meshcat.SetProperty("/Background", "visible", false);
  @endverbatim
  will turn off the background.

  Note: Meshcat appends an extra path element with the name <object> to every
  item created with SetObject, so if you want to modify a property of the object
  itself, rather than the group containing it, you should ensure that your path
  is of the form /meshcat/foo/<object>.

  @param path a "/"-separated string indicating the path in the scene tree.
  @param property the string name of the property to set
  @param value the new value.
  */
  void SetProperty(std::string_view path, const std::string& property,
                   bool value);

  /** Sets a single named property of the object at the given path.

  Note: Meshcat appends an extra path element with the name <object> to every
  item created with SetObject, so if you want to modify a property of the object
  itself, rather than the group containing it, you should ensure that your path
  is of the form /meshcat/foo/<object>. For example,
  @verbatim
  meshcat.SetProperty("/Cameras/default/rotated/<object>", "zoom", 2.0);
  meshcat.SetProperty("/Lights/DirectionalLight/<object>", "intensity", 1.0);
  @endverbatim

  @param path a "/"-separated string indicating the path in the scene tree.
  @param property the string name of the property to set
  @param value the new value.
  */
  void SetProperty(std::string_view path, const std::string& property,
                   double value);

  // TODO(russt): Implement SetAnimation().
  // TODO(russt): Implement SetButton() and SetSlider() as wrappers on
  // set_control.

  /** @name (Advanced)

  These methods are intended to primarily for testing. These calls
  must safely acquire the data from the websocket thread and will block
  execution waiting for that data to be acquired. */
  //@{

  /** Returns true iff `path` exists in the current internal (server) tree.
  Note that the javascript client maintains its own tree, which may contain
  elements that are not in the server tree. */
  bool HasPath(std::string_view path) const;

  /** Returns the msgpack representation of the object stored in the internal
  (server) tree at `path`, or the empty string if `path` does not exist or no
  object has been set at `path`. */
  std::string GetPackedObject(std::string_view path) const;

  /** Returns the msgpack representation of the transform stored in the internal
  (server) tree at `path`, or the empty string if `path` does not exist or no
  transform has been set at `path`. */
  std::string GetPackedTransform(std::string_view path) const;

  /** Returns the msgpack representation of the `property` stored in the
  internal (server) tree at `path`, or the empty string if `path` does not
  exist or `property` has not been set at `path`. */
  std::string GetPackedProperty(std::string_view path,
                                const std::string& property) const;
  //@}
 private:
  // Provides PIMPL encapsulation of websocket types.
  class WebSocketPublisher;
  std::unique_ptr<WebSocketPublisher> publisher_;
};

}  // namespace geometry
}  // namespace drake
