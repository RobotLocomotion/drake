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
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"

namespace {
std::string LoadResource(const std::string& resource_name) {
  const std::string resource = drake::FindResourceOrThrow(resource_name);
  std::ifstream file(resource.c_str(), std::ios::in);
  if (!file.is_open())
    throw std::runtime_error("Error opening resource: " + resource_name);
  std::stringstream content;
  content << file.rdbuf();
  file.close();
  return content.str();
}

const std::string& GetUrlContent(std::string_view url_path) {
  static const drake::never_destroyed<std::string> main_min_js(
      LoadResource("drake/external/meshcat/dist/main.min.js"));
  static const drake::never_destroyed<std::string> favicon_ico(
      LoadResource("drake/doc/favicon.ico"));
  static const drake::never_destroyed<std::string> index_html(
      LoadResource("drake/external/meshcat/dist/index.html"));
  if (url_path == "/main.min.js") {
    return main_min_js.access();
  }
  if (url_path == "/favicon.ico") {
    return favicon_ico.access();
  }
  return index_html.access();
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

  WebSocketPublisher()
      : main_thread_id_(std::this_thread::get_id()),
        app_future_(app_promise_.get_future()) {}

  void SetWebSocketThreadId(std::thread::id id) {
    websocket_thread_id_ = id;
  }

  // Call this from websocket thread.
  void SetAppPromise(uWS::App* app, uWS::Loop* loop, int port,
                     us_listen_socket_t* listen_socket) {
    DRAKE_DEMAND(std::this_thread::get_id() == websocket_thread_id_);
    app_promise_.set_value(std::make_tuple(app, loop, port, listen_socket));
  }

  // Call this from main thread.
  void GetAppFuture() {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    std::tie(app_, loop_, port_, listen_socket_) = app_future_.get();
  }

  int port() const {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(port_ > 0);
    return port_;
  }

  template <typename T>
  void SetProperty(std::string_view path, std::string_view property,
                   const T& value) {
    DRAKE_DEMAND(app_ != nullptr);
    DRAKE_DEMAND(loop_ != nullptr);
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);

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
      set_properties_[fmt::format("{}/{}", path, property)] = msg;
    });
  }

  void SendTree(WebSocket* ws) {
    DRAKE_DEMAND(std::this_thread::get_id() == websocket_thread_id_);
    // TODO(russt): Generalize this to publishing the entire scene tree.
    for (const auto& [key, msg] : set_properties_) {
      unused(key);
      ws->send(msg);
    }
  }

  void StartShutdown() {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    // Pass by value.
    loop_->defer([socket = listen_socket_]() {
      us_listen_socket_close(0, socket);
    });
  }

 private:
  std::promise<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
      app_promise_{};

  // These variables should only be accessed in the main thread.
  std::thread::id main_thread_id_{};
  std::future<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
      app_future_{};
  int port_{-1};
  us_listen_socket_t* listen_socket_{nullptr};

  // These variables should only be accessed in the websocket thread.
  std::thread::id websocket_thread_id_{};
  uWS::App* app_{nullptr};
  std::map<std::string, std::string> set_properties_{};

  // Only loop_->defer() should be called from outside the websocket_thread. See
  // the documentation for uWebSockets for further details:
  // https://github.com/uNetworking/uWebSockets/blob/d94bf2cd43bed5e0de396a8412f156e15c141e98/misc/READMORE.md#threading
  uWS::Loop* loop_{nullptr};
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
  publisher_->StartShutdown();
  websocket_thread_.join();
}

std::string Meshcat::web_url() const {
  return fmt::format("http://localhost:{}", publisher_->port());
}

std::string Meshcat::ws_url() const {
  return fmt::format("ws://localhost:{}", publisher_->port());
}

void Meshcat::JoinWebSocketThread() { websocket_thread_.join(); }

void Meshcat::SetProperty(std::string_view path, std::string_view property,
                          bool value) {
  publisher_->SetProperty(path, property, value);
}

void Meshcat::WebsocketMain() {
  publisher_->SetWebSocketThreadId(std::this_thread::get_id());

  int port = 7001;
  const int kMaxPort = 7099;

  uWS::App::WebSocketBehavior<PerSocketData> behavior;
  behavior.open = [this](WebSocket* ws) {
    ws->subscribe("all");
    // Update this new connection with previously published data.
    publisher_->SendTree(ws);
  };

  // Fetch the index once to be sure that we preload the content.
  GetUrlContent("/");

  uWS::App app =
      uWS::App()
          .get("/*",
               [&](uWS::HttpResponse<SSL>* res, uWS::HttpRequest* req) {
                 res->end(GetUrlContent(req->getUrl()));
               })
          .ws<PerSocketData>("/*", std::move(behavior));

  us_listen_socket_t* listen_socket = nullptr;
  do {
    app.listen(
        port, LIBUS_LISTEN_EXCLUSIVE_PORT,
        [port, &listen_socket](us_listen_socket_t* socket) {
          if (socket) {
            drake::log()->info(
                "Meshcat listening for connections at http://localhost:{}",
                port);
            listen_socket = socket;
          }
        });
  } while (listen_socket == nullptr && port++ < kMaxPort);

  if (listen_socket == nullptr) {
    throw std::runtime_error("Meshcat failed to open a websocket port.");
  }

  publisher_->SetAppPromise(&app, uWS::Loop::get(), port, listen_socket);

  app.run();
}

}  // namespace geometry
}  // namespace drake
