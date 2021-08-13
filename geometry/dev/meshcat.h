#pragma once

#include <future>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <App.h>

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

  /** Constructs the Meshcat instance. */
  Meshcat();

  /** The thread run by this class will run for the lifetime of the Meshcat
  instance.  We provide this method as a simple way to block the main thread
  to allow users to open their browser and work with the Meshcat scene. */
  void join_websocket_thread();

  /** Forwards a set_property(...) message to the meshcat viewers. For example,
  @verbatim
  meshcat.SetProperty("/Background", "visible", false);
  @endverbatim
  will turn off the background. */
  void SetProperty(const std::string& path, const std::string& property,
                   bool value);

 private:
  void WebsocketMain(
      std::promise<std::pair<uWS::App*, uWS::Loop*>> app_promise);

  std::thread websocket_thread_{};

  // Only loop_->defer() should be called from outside the websocket_thread. See
  // the documentation for uWebSockets for further details:
  // https://github.com/uNetworking/uWebSockets/blob/d94bf2cd43bed5e0de396a8412f156e15c141e98/misc/READMORE.md#threading
  uWS::Loop* loop_{nullptr};

  // The remaining variables should only be accessed from the websocket_thread.
  uWS::App* app_{nullptr};
  std::unordered_map<std::string, std::string> set_properties_;
};

}  // namespace geometry
}  // namespace drake
