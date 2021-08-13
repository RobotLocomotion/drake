#pragma once

#include <memory>
#include <string>
#include <thread>

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
the viewer.  It is the result of the first PR in a train of PRs that will
establish the full Meshcat functionality.  See #13038.
*/
class Meshcat {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Meshcat)

  /** Constructs the Meshcat instance. It will listen on the first available
  port starting at 7001 (up to 7099). */
  Meshcat();

  ~Meshcat();

  /** The thread run by this class will run for the lifetime of the Meshcat
  instance.  We provide this method as a simple way to block the main thread
  to allow users to open their browser and work with the Meshcat scene. */
  void JoinWebSocketThread();

  /** Forwards a set_property(...) message to the meshcat viewers. For example,
  @verbatim
  meshcat.SetProperty("/Background", "visible", false);
  @endverbatim
  will turn off the background. */
  void SetProperty(const std::string& path, const std::string& property,
                   bool value);

 private:
  void WebsocketMain();
  std::thread websocket_thread_{};

  // Provides PIMPL encapsulation of websocket types.
  class WebSocketPublisher;
  std::unique_ptr<WebSocketPublisher> publisher_;
};

}  // namespace geometry
}  // namespace drake
