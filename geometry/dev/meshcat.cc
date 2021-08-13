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

Meshcat::Meshcat() {
  // Note: uWS::Loop::get() returns a thread_local loop, so I must obtain the
  // loop instance from the websocket thread.
  std::promise<std::pair<uWS::App*, uWS::Loop*>> app_promise;
  std::future<std::pair<uWS::App*, uWS::Loop*>> app_future =
      app_promise.get_future();
  websocket_thread_ =
      std::thread(&Meshcat::WebsocketMain, this, std::move(app_promise));
  std::tie(app_, loop_) = app_future.get();
}

void Meshcat::join_websocket_thread() { websocket_thread_.join(); }

void Meshcat::SetProperty(const std::string& path, const std::string& property,
                          bool) {
  DRAKE_ASSERT(app_ != nullptr);

  std::stringstream message;
  msgpack::pack(
      message,
      std::unordered_map<std::string, std::string>(
          {{"type", "set_property"}, {"path", path}, {"property", property}}));
  msgpack::pack(message,
                std::unordered_map<std::string, bool>({{"value", false}}));

  // Note: Must pass message by value, since it will go out of scope.
  loop_->defer([this, msg = message.str(), path, property]() {
    app_->publish("all", msg, uWS::OpCode::BINARY, false);
    set_properties_[path + "/" + property] = msg;
  });
}

void Meshcat::WebsocketMain(
    std::promise<std::pair<uWS::App*, uWS::Loop*>> app_promise) {
  // Preload the three files we will serve (always).
  static const drake::never_destroyed<std::string> index_html(
      LoadFile("drake/external/meshcat/dist/index.html"));
  static const drake::never_destroyed<std::string> main_min_js(
      LoadFile("drake/external/meshcat/dist/main.min.js"));
  static const drake::never_destroyed<std::string> favicon_ico(
      LoadFile("drake/doc/favicon.ico"));
  int port = 7001;
  const int kMaxPort = 7050;

  constexpr static bool SSL = false;
  struct PerSocketData {
    // Intentionally left empty.
  };
  using WebSocket = uWS::WebSocket<SSL, true, PerSocketData>;

  uWS::App::WebSocketBehavior<PerSocketData> behavior;
  behavior.open = [this](WebSocket* ws) {
    ws->subscribe("all");

    // Update this new connection with previously published data.
    // TODO(russt): Generalize this to publishing the entire scene tree.
    for (const auto& [key, msg] : set_properties_) {
      unused(key);
      ws->send(msg);
    }
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

  app_promise.set_value(std::make_pair(&app, uWS::Loop::get()));

  app.run();

  // run() should not terminate.  If it does, then we've failed.
  throw std::runtime_error("Meshcat websocket thread failed");
}

}  // namespace geometry
}  // namespace drake
