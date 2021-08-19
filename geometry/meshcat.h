#pragma once

#include <memory>
#include <string>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace geometry {

/** Provides an interface to https://github.com/rdeits/meshcat.

Each instance of this class spawns a thread which runs an http/websocket server.
Users can navigate their browser to the hosted URL to visualize the Meshcat
scene.  Note that, unlike many visualizers, one cannot open the visualizer until
this server is running.

Note: This code is currently a skeleton implementation; it only allows you to
set (boolean) properties as a minimal demonstration of sending data from C++ to
the viewer.  It is the result of the second PR in a train of PRs that will
establish the full Meshcat functionality. See #13038.
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

  /** Forwards a set_property(...) message to the meshcat viewers. For example,
  @verbatim
  meshcat.SetProperty("/Background", "visible", false);
  @endverbatim
  will turn off the background. */
  void SetProperty(std::string_view path, std::string_view property,
                   bool value);

 private:
  // Provides PIMPL encapsulation of websocket types.
  class WebSocketPublisher;
  std::unique_ptr<WebSocketPublisher> publisher_;
};

}  // namespace geometry
}  // namespace drake
