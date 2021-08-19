#include "drake/geometry/meshcat.h"

#include <fstream>
#include <future>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include <App.h>
#include <fmt/format.h>
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
  void SetAppPromise(uWS::App* app, uWS::Loop* loop, int port,
                     us_listen_socket_t* listen_socket) {
    app_promise.set_value(std::make_tuple(app, loop, port, listen_socket));
  }

  // Call this from main thread.
  void GetAppFuture() {
    std::tie(app_, loop_, port_, listen_socket_) = app_future.get();
  }

  int port() const {
    DRAKE_DEMAND(port_ > 0);
    return port_;
  }

  template <typename T>
  void SetProperty(const std::string& path, const std::string& property,
                   const T& value) {
    DRAKE_ASSERT(app_ != nullptr);
    DRAKE_ASSERT(loop_ != nullptr);

    std::stringstream message;

    msgpack::zone z;
    msgpack::pack(message, std::map<std::string, msgpack::object>(
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

  void Shutdown() {
    // This can be extremely slow.  See discussion at
    // https://github.com/uNetworking/uWebSockets/discussions/809
    us_listen_socket_close(0, listen_socket_);
  }

 private:
  std::promise<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
      app_promise{};
  std::future<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
      app_future{};

  int port_{-1};
  us_listen_socket_t* listen_socket_;

  // Only loop_->defer() should be called from outside the websocket_thread. See
  // the documentation for uWebSockets for further details:
  // https://github.com/uNetworking/uWebSockets/blob/d94bf2cd43bed5e0de396a8412f156e15c141e98/misc/READMORE.md#threading
  uWS::Loop* loop_{nullptr};

  // The remaining variables should only be accessed from the websocket_thread.
  uWS::App* app_{nullptr};
  std::map<std::string, std::string> set_properties_{};
};

Meshcat::Meshcat() {
  // A std::promise is made in the WebSocketPublisher.
  publisher_ = std::make_unique<WebSocketPublisher>();
  websocket_thread_ = std::thread(&Meshcat::WebsocketMain, this);
  // The std::promise is full-filled in WebsocketMain; we wait here to obtain
  // that value.
  publisher_->GetAppFuture();
}

Meshcat::~Meshcat() {
  publisher_->Shutdown();
  JoinWebSocketThread();
}

std::string Meshcat::web_url() const {
  return fmt::format("http://localhost:{}", publisher_->port());
}

std::string Meshcat::ws_url() const {
  return fmt::format("ws://localhost:{}", publisher_->port());
}

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

  us_listen_socket_t* listen_socket = nullptr;
  do {
    app.listen(
        port, LIBUS_LISTEN_EXCLUSIVE_PORT,
        [port, &listen_socket](us_listen_socket_t* socket) {
          if (socket) {
            std::cout
                << "Meshcat listening for connections at http://127.0.0.1:"
                << port << std::endl;
            listen_socket = socket;
          }
        });
  } while (listen_socket == nullptr && port++ <= kMaxPort);

  publisher_->SetAppPromise(&app, uWS::Loop::get(), port, listen_socket);

  app.run();
}

}  // namespace geometry
}  // namespace drake
