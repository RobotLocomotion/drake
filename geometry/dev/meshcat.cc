#include "drake/geometry/dev/meshcat.h"

#include <fstream>
#include <future>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <App.h>
#include <msgpack.hpp>

#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/unused.h"

namespace {
std::string LoadFile(const std::string& filename) {
  const std::string resource = drake::FindResourceOrThrow(filename);
  std::ifstream file(resource.c_str(), std::ios::in);
  if (!file.is_open())
    throw std::runtime_error("Error opening file: " + filename);
  std::stringstream content;
  content << file.rdbuf();
  file.close();
  return content.str();
}
}  // namespace

namespace drake {
namespace geometry {

namespace {

constexpr static bool SSL = false;
struct PerSocketData {
  // Intentionally left empty.
};
using WebSocket = uWS::WebSocket<SSL, true, PerSocketData>;

}  // namespace

class Meshcat::WebSocketPublisher {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WebSocketPublisher);

  WebSocketPublisher() : app_future(app_promise.get_future()) {}

  // Call this from websocket thread.
  void SetAppPromise(uWS::App* app, uWS::Loop* loop) {
    app_promise.set_value(std::make_pair(app, loop));
  }

  // Call this from main thread.
  void GetAppFuture() {
    std::tie(app_, loop_) = app_future.get();
  }

  template <typename T>
  void SetProperty(const std::string& path, const std::string& property,
                   const T& value) {
    DRAKE_ASSERT(app_ != nullptr);
    DRAKE_ASSERT(loop_ != nullptr);

    std::stringstream message;

    msgpack::zone z;
    msgpack::pack(message, std::unordered_map<std::string, msgpack::object>(
                              {{"type", msgpack::object("set_property", z)},
                                {"path", msgpack::object(path, z)},
                                {"property", msgpack::object(property, z)},
                                {"value", msgpack::object(value, z)}}));

    // Note: Must pass path and property by value because they will go out of
    // scope.
    loop_->defer([this, path, property, msg = message.str()]() {
      app_->publish("all", msg, uWS::OpCode::BINARY, false);
      set_properties_[path + "/" + property] = msg;
    });
  }

  void SendTree(WebSocket* ws) {
    // TODO(russt): Generalize this to publishing the entire scene tree.
    for (const auto& [key, msg] : set_properties_) {
      unused(key);
      ws->send(msg);
    }
  }

 private:
  std::promise<std::pair<uWS::App*, uWS::Loop*>> app_promise{};
  std::future<std::pair<uWS::App*, uWS::Loop*>> app_future{};

  // Only loop_->defer() should be called from outside the websocket_thread. See
  // the documentation for uWebSockets for further details:
  // https://github.com/uNetworking/uWebSockets/blob/d94bf2cd43bed5e0de396a8412f156e15c141e98/misc/READMORE.md#threading
  uWS::Loop* loop_{nullptr};

  // The remaining variables should only be accessed from the websocket_thread.
  uWS::App* app_{nullptr};
  std::unordered_map<std::string, std::string> set_properties_{};
};

Meshcat::Meshcat() {
  // A std::promise is made in the WebSocketPublisher.
  publisher_ = std::make_unique<WebSocketPublisher>();
  websocket_thread_ = std::thread(&Meshcat::WebsocketMain, this);
  // The std::promise is full-filled in WebsocketMain; we wait here to obtain
  // that value.
  publisher_->GetAppFuture();
}

Meshcat::~Meshcat() = default;

void Meshcat::JoinWebSocketThread() { websocket_thread_.join(); }

void Meshcat::SetProperty(const std::string& path, const std::string& property,
                          bool value) {
  publisher_->SetProperty(path, property, value);
}

void Meshcat::WebsocketMain() {
  // Preload the three files we will serve (always).
  static const drake::never_destroyed<std::string> index_html(
      LoadFile("drake/external/meshcat/dist/index.html"));
  static const drake::never_destroyed<std::string> main_min_js(
      LoadFile("drake/external/meshcat/dist/main.min.js"));
  static const drake::never_destroyed<std::string> favicon_ico(
      LoadFile("drake/doc/favicon.ico"));
  int port = 7001;
  const int kMaxPort = 7099;

  uWS::App::WebSocketBehavior<PerSocketData> behavior;
  behavior.open = [this](WebSocket* ws) {
    ws->subscribe("all");
    // Update this new connection with previously published data.
    publisher_->SendTree(ws);
  };

  uWS::App app =
      uWS::App()
          .get("/*",
               [&](uWS::HttpResponse<SSL>* res, uWS::HttpRequest* req) {
                 if (req->getUrl() == "/main.min.js") {
                   res->end(main_min_js.access());
                 } else if (req->getUrl() == "/favicon.ico") {
                   res->end(favicon_ico.access());
                 } else {
                   res->end(index_html.access());
                 }
               })
          .ws<PerSocketData>("/*", std::move(behavior));

  bool listening = false;
  do {
    app.listen(
        port, LIBUS_LISTEN_EXCLUSIVE_PORT,
        [port, &listening](us_listen_socket_t* listenSocket) {
          if (listenSocket) {
            std::cout
                << "Meshcat listening for connections at http://127.0.0.1:"
                << port << std::endl;
            listening = true;
          }
        });
  } while (!listening && port++ <= kMaxPort);

  publisher_->SetAppPromise(&app, uWS::Loop::get());

  app.run();

  // run() should not terminate.  If it does, then we've failed.
  throw std::runtime_error("Meshcat websocket thread failed");
}

}  // namespace geometry
}  // namespace drake
