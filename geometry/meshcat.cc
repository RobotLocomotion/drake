#include "drake/geometry/meshcat.h"

#include <exception>
#include <fstream>
#include <future>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include <App.h>
#include <fmt/format.h>
#include <msgpack.hpp>
#include <uuid.h>

#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/meshcat_types.h"

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
      LoadResource("drake/geometry/meshcat.html"));
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

using math::RigidTransformd;
using math::RotationMatrixd;

constexpr static bool kSsl = false;
constexpr static bool kIsServer = true;
struct PerSocketData {
  // Intentionally left empty.
};
using WebSocket = uWS::WebSocket<kSsl, kIsServer, PerSocketData>;
using MsgPackMap = std::map<std::string, msgpack::object>;

// The maximum "backpressure" allowed each websocket, in bytes.  This
// effectively sets a maximum size for messages, which may include mesh files
// and texture maps.  50mb is a guess at a compromise that is safely larger than
// reasonable meshfiles.
constexpr static double kMaxBackPressure{50 * 1024 * 1024};

class SceneTreeElement {
 public:
  // Member access methods (object_, transform_, and properties_ should be
  // effectively public).
  const std::optional<std::string>& object() const { return object_; }
  std::optional<std::string>& object() { return object_; }
  const std::optional<std::string>& transform() const { return transform_; }
  std::optional<std::string>& transform() { return transform_; }
  const std::map<std::string, std::string>& properties() const {
    return properties_;
  }
  std::map<std::string, std::string>& properties() { return properties_; }

  // Returns this element or a descendant, based on a recursive evaluation of
  // the `path`.  Adds new elements if they do not exist.  Comparable to
  // std::map::operator[].
  SceneTreeElement& operator[](std::string_view path) {
    while (!path.empty() && path.front() == '/') {
      path.remove_prefix(1);
    }
    if (path.empty()) {
      return *this;
    }
    auto loc = path.find_first_of("/");
    std::string name(path.substr(0, loc));
    auto child = children_.find(name);
    // Create the child if it doesn't exist.
    if (child == children_.end()) {
      child =
          children_.emplace(name, std::make_unique<SceneTreeElement>()).first;
    }
    if (loc == std::string_view::npos) {
      return *child->second;
    } else {
      return (*child->second)[path.substr(loc + 1)];
    }
  }

  // Returns a pointer to `this` element or a descendant, based on a recursive
  // evaluation of the `path`, or nullptr if `path` does not exist.
  const SceneTreeElement* Find(std::string_view path) const {
    while (!path.empty() && path.front() == '/') {
      path.remove_prefix(1);
    }
    if (path.empty()) {
      return this;
    }
    auto loc = path.find_first_of("/");
    std::string name(path.substr(0, loc));
    auto child = children_.find(name);
    if (child == children_.end()) {
      return nullptr;
    }
    if (loc == std::string_view::npos) {
      return child->second.get();
    } else {
      return child->second->Find(path.substr(loc + 1));
    }
  }

  // Deletes `path` from the tree.  See Meshcat::Delete.
  void Delete(std::string_view path) {
    while (!path.empty() && path.front() == '/') {
      path.remove_prefix(1);
    }
    if (path.empty()) {
      // To match javascript, we don't delete the empty path.
      return;
    }

    auto loc = path.find_first_of("/");
    auto child = children_.find(std::string(path.substr(0, loc)));
    if (child == children_.end()) {
      return;
    }
    if (loc == std::string_view::npos) {
      children_.erase(child);
      return;
    }
    child->second->Delete(path.substr(loc + 1));
  }

  // Sends the entire tree on `ws`.
  void Send(WebSocket* ws) {
    if (object_) {
      ws->send(*object_);
    }
    if (transform_) {
      ws->send(*transform_);
    }
    for (const auto& [property, msg] : properties_) {
      unused(property);
      ws->send(msg);
    }

    for (const auto& [name, child] : children_) {
      unused(name);
      child->Send(ws);
    }
  }

 private:
  // Note: We use std::optional here to clearly denote the variables that have
  // not been set, and therefore need not be sent over the websocket.

  // The msgpack'd set_object command.
  std::optional<std::string> object_{std::nullopt};
  // The msgpack'd set_transform command.
  std::optional<std::string> transform_{std::nullopt};
  // The msgpack'd set_property command(s).
  std::map<std::string, std::string> properties_{};
  // Children, with the key value denoting their (relative) path name.
  std::map<std::string, std::unique_ptr<SceneTreeElement>> children_{};
};

class MeshcatShapeReifier : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatShapeReifier);

  explicit MeshcatShapeReifier(std::string uuid)
      : uuid_(std::move(uuid)) {}

  ~MeshcatShapeReifier() = default;

  const std::string& uuid() const { return uuid_; }

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void* data) override {
    DRAKE_ASSERT(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    internal::GeometryData& geometry = lumped.geometries[0];

    geometry.uuid = uuid_;
    geometry.type = "SphereGeometry";
    geometry.radius = sphere.radius();
    geometry.widthSegments = 20;
    geometry.heightSegments = 20;
  }

  void ImplementGeometry(const Cylinder& cylinder, void* data) override {
    DRAKE_ASSERT(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    internal::GeometryData& geometry = lumped.geometries[0];

    geometry.uuid = uuid_;
    geometry.type = "CylinderGeometry";
    geometry.radiusBottom = cylinder.radius();
    geometry.radiusTop = cylinder.radius();
    geometry.height = cylinder.length();
    geometry.radialSegments = 50;

    Eigen::Map<Eigen::Matrix4d>(lumped.object.matrix) =
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0))
            .GetAsMatrix4();
  }

  void ImplementGeometry(const HalfSpace&, void*) override {
    // TODO(russt): Use PlaneGeometry with fields width, height,
    // widthSegments, heightSegments
    drake::log()->warn("Meshcat does not display HalfSpace geometry (yet).");
  }

  void ImplementGeometry(const Box& box, void* data) override {
    DRAKE_ASSERT(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    internal::GeometryData& geometry = lumped.geometries[0];

    geometry.uuid = uuid_;
    geometry.type = "BoxGeometry";
    geometry.width = box.width();
    // Three.js uses height for the y axis; Drake uses depth.
    geometry.height = box.depth();
    geometry.depth = box.height();
  }

  void ImplementGeometry(const Capsule&, void*) override {
    drake::log()->warn("Meshcat does not display Capsule geometry (yet).");
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* data) override {
    // Implemented as a Sphere stretched by a diagonal transform.
    DRAKE_ASSERT(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    internal::GeometryData& geometry = lumped.geometries[0];

    geometry.uuid = uuid_;
    geometry.type = "SphereGeometry";
    geometry.radius = 1;
    geometry.widthSegments = 20;
    geometry.heightSegments = 20;

    Eigen::Map<Eigen::Matrix4d> matrix(lumped.object.matrix);
    matrix(0, 0) = ellipsoid.a();
    matrix(1, 1) = ellipsoid.b();
    matrix(2, 2) = ellipsoid.c();
  }

  template <typename T>
  void ImplementMesh(const T& mesh, void* data) {
    DRAKE_ASSERT(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    internal::GeometryData& geometry = lumped.geometries[0];

    // TODO(russt): Use file contents to generate the uuid, and avoid resending
    // meshes unless necessary.  Using the filename is tempting, but that leads
    // to problems when the file contents change on disk.

    size_t pos = mesh.filename().find_last_of('.');
    if (pos == std::string::npos) {
      drake::log()->warn("Meshcat: Unsupported extension for mesh filename {}",
                         mesh.filename());
      return;
    }
    geometry.uuid = uuid_;
    geometry.format = mesh.filename().substr(pos + 1);

    std::ifstream input(mesh.filename(), std::ios::binary | std::ios::ate);
    if (!input.is_open()) {
      drake::log()->warn("Meshcat: Could not open mesh filename {}",
                         mesh.filename());
      return;
    }

    // We simply dump the binary contents of the file into the data field of the
    // message.  The javascript meshcat takes care of the rest.
    int size = input.tellg();
    if (size > kMaxBackPressure) {
      throw std::runtime_error(fmt::format(
          "The meshfile at {} is too large for the current websocket setup.  "
          "Size {} is greater than the max backpressure {}.  You will either "
          "need to reduce the size of your mesh, or modify the code to "
          "increase the allowance.",
          mesh.filename(), size, kMaxBackPressure));
    }
    input.seekg(0, std::ios::beg);
    geometry.data.reserve(size);
    geometry.data.assign((std::istreambuf_iterator<char>(input)),
                         std::istreambuf_iterator<char>());

    // TODO(russt): Implement textures.  Need to add LumpedData.textures,
    // LumpedData.images, etc.
    geometry.type = "_meshfile_geometry";

    Eigen::Map<Eigen::Matrix4d> matrix(lumped.object.matrix);
    matrix(0, 0) = mesh.scale();
    matrix(1, 1) = mesh.scale();
    matrix(2, 2) = mesh.scale();
  }

  void ImplementGeometry(const Mesh& mesh, void* data) override {
    ImplementMesh(mesh, data);
  }

  void ImplementGeometry(const Convex& mesh, void* data) override {
    ImplementMesh(mesh, data);
  }

 private:
  std::string uuid_;
};

}  // namespace

class Meshcat::WebSocketPublisher {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WebSocketPublisher);

  WebSocketPublisher()
      : prefix_("/drake"), main_thread_id_(std::this_thread::get_id()) {
    std::promise<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
        app_promise;
    std::future<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
        app_future = app_promise.get_future();
    websocket_thread_ = std::thread(&WebSocketPublisher::WebSocketMain, this,
                                    std::move(app_promise));
    std::tie(app_, loop_, port_, listen_socket_) = app_future.get();
  }

  ~WebSocketPublisher() {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    loop_->defer([this]() {
      us_listen_socket_close(0, listen_socket_);
      for (auto* ws : websockets_) {
        ws->close();
      }
    });
    websocket_thread_.join();
  }

  int port() const {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    return port_;
  }

  void SetObject(std::string_view path, const Shape& shape,
                        const Rgba& rgba) {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(app_ != nullptr);
    DRAKE_DEMAND(loop_ != nullptr);

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetObjectData data;
    data.path = FullPath(path);

    // TODO(russt): This current meshcat set_object interface couples geometry,
    // material, and object for convenience, but we might consider decoupling
    // them again for efficiency. We don't want to send meshes over the network
    // (which could be from the cloud to a local browser) more than necessary.

    internal::MaterialData& material = data.object.materials[0];
    material.uuid = uuids::to_string(uuid_generator());
    material.type = "MeshPhongMaterial";
    material.color = (static_cast<int>(255 * rgba.r()) << 16) +
                     (static_cast<int>(255 * rgba.g()) << 8) +
                     static_cast<int>(255 * rgba.b());
    // From meshcat-python: Three.js allows a material to have an opacity
    // which is != 1, but to still be non - transparent, in which case the
    // opacity only serves to desaturate the material's color. That's a
    // pretty odd combination of things to want, so by default we just use
    // the opacity value to decide whether to set transparent to True or
    // False.
    material.transparent = (rgba.a() != 1.0);
    material.opacity = rgba.a();

    internal::Object3dData& object3d = data.object.object;
    object3d.uuid = uuids::to_string(uuid_generator());
    object3d.type = "Mesh";
    object3d.material = material.uuid;

    MeshcatShapeReifier reifier(uuids::to_string(uuid_generator()));
    shape.Reify(&reifier, &data.object);
    if (data.object.geometries[0].type.empty()) {
      // Then this shape is not supported, and I should not send the message,
      // nor add the object to the tree.
      return;
    }
    object3d.geometry = reifier.uuid();

    // Note: pass all temporaries by value.
    loop_->defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  void SetTransform(std::string_view path,
                    const RigidTransformd& X_ParentPath) {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(app_ != nullptr);
    DRAKE_DEMAND(loop_ != nullptr);

    internal::SetTransformData data;
    data.path = FullPath(path);
    Eigen::Map<Eigen::Matrix4d>(data.matrix) = X_ParentPath.GetAsMatrix4();

    // Note: pass all temporaries by value.
    loop_->defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.transform() = std::move(message);
    });
  }

  void Delete(std::string_view path) {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(app_ != nullptr);
    DRAKE_DEMAND(loop_ != nullptr);

    internal::DeleteData data;
    data.path = FullPath(path);

    // Note: pass all temporaries by value.
    loop_->defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      scene_tree_root_.Delete(data.path);
    });
  }

  template <typename T>
  void SetProperty(std::string_view path, std::string property,
                   const T& value) {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(app_ != nullptr);
    DRAKE_DEMAND(loop_ != nullptr);

    internal::SetPropertyData<T> data;
    data.path = FullPath(path);
    data.property = std::move(property);
    data.value = value;

    // Note: pass all temporaries by value.
    loop_->defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.properties()[data.property] = std::move(message);
    });
  }

  bool HasPath(std::string_view path) const {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<bool> p;
    std::future<bool> f = p.get_future();
    // Note: pass all temporaries by value.
    loop_->defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      p.set_value(scene_tree_root_.Find(path) != nullptr);
    });
    return f.get();
  }

  std::string GetPackedObject(std::string_view path) const {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    // Note: pass all temporaries by value.
    loop_->defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      const SceneTreeElement* e = scene_tree_root_.Find(path);
      if (!e || !e->object()) {
        p.set_value("");
      } else {
        p.set_value(*e->object());
      }
    });
    return f.get();
  }

  std::string GetPackedTransform(std::string_view path) const {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    // Note: pass all temporaries by value.
    loop_->defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      const SceneTreeElement* e = scene_tree_root_.Find(path);
      if (!e || !e->transform()) {
        p.set_value("");
      } else {
        p.set_value(*e->transform());
      }
    });
    return f.get();
  }

  std::string GetPackedProperty(std::string_view path,
                                std::string property) const {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    // Note: pass all temporaries by value.
    loop_->defer([this, path = FullPath(path), property = std::move(property),
                  p = std::move(p)]() mutable {
      const SceneTreeElement* e = scene_tree_root_.Find(path);
      if (!e) {
        p.set_value("");
      } else {
        auto prop = e->properties().find(property);
        if (prop == e->properties().end()) {
          p.set_value("");
        } else {
          p.set_value(prop->second);
        }
      }
    });
    return f.get();
  }

 private:
  void WebSocketMain(
      std::promise<std::tuple<uWS::App*, uWS::Loop*, int, us_listen_socket_t*>>
          app_promise) {
    websocket_thread_id_ = std::this_thread::get_id();

    int port = 7001;
    const int kMaxPort = 7099;

    uWS::App::WebSocketBehavior<PerSocketData> behavior;
    behavior.open = [this](WebSocket* ws) {
      ws->subscribe("all");
      // Update this new connection with previously published data.
      SendTree(ws);
      websockets_.emplace(ws);
    };
    // TODO(russt): I could increase this more if necessary (when it was too
    // low, some SetObject messages were dropped).  But at some point the real
    // fix is to actually throttle the sending (e.g. by slowing down the main
    // thread).
    behavior.maxBackpressure = kMaxBackPressure;
    behavior.close = [this](WebSocket* ws, int, std::string_view) {
      websockets_.erase(ws);
    };

    uWS::App app =
        uWS::App()
            .get("/*",
                 [&](uWS::HttpResponse<kSsl>* res, uWS::HttpRequest* req) {
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

    app_promise.set_value(
        std::make_tuple(&app, uWS::Loop::get(), port, listen_socket));

    app.run();
  }

  void SendTree(WebSocket* ws) {
    DRAKE_DEMAND(std::this_thread::get_id() == websocket_thread_id_);
    scene_tree_root_.Send(ws);
  }

  std::string FullPath(std::string_view path) const {
    while (path.size() > 1 && path.back() == '/') {
      path.remove_suffix(1);
    }
    if (path.empty()) {
      return prefix_;
    }
    if (path.front() == '/') {
      return std::string(path);
    }
    return fmt::format("{}/{}", prefix_, path);
  }

  std::thread websocket_thread_{};
  const std::string prefix_{};

  // These variables should only be accessed in the main thread, where "main
  // thread" is the thread in which this class was constructed.
  std::thread::id main_thread_id_{};
  int port_{};
  std::mt19937 generator_{};

  // These variables should only be accessed in the websocket thread.
  std::thread::id websocket_thread_id_{};
  SceneTreeElement scene_tree_root_{};

  // These pointers should only be accessed in the main thread, but the objects
  // they are pointing to should be only used in the websocket thread.
  uWS::App* app_{nullptr};
  us_listen_socket_t* listen_socket_{nullptr};
  std::set<WebSocket*> websockets_{};

  // This pointer should only be accessed in the main thread, but the Loop
  // object itself should be only used in the websocket thread, with one
  // exception: loop_->defer(), which is thread safe. See the documentation for
  // uWebSockets for further details:
  // https://github.com/uNetworking/uWebSockets/blob/d94bf2cd43bed5e0de396a8412f156e15c141e98/misc/READMORE.md#threading
  uWS::Loop* loop_{nullptr};
};

Meshcat::Meshcat() {
  // Fetch the index once to be sure that we preload the content.
  GetUrlContent("/");

  publisher_ = std::make_unique<WebSocketPublisher>();
}

Meshcat::~Meshcat() = default;

std::string Meshcat::web_url() const {
  return fmt::format("http://localhost:{}", publisher_->port());
}

std::string Meshcat::ws_url() const {
  return fmt::format("ws://localhost:{}", publisher_->port());
}

void Meshcat::SetObject(std::string_view path, const Shape& shape,
                        const Rgba& rgba) {
  publisher_->SetObject(path, shape, rgba);
}

void Meshcat::SetTransform(std::string_view path,
                           const RigidTransformd& X_ParentPath) {
  publisher_->SetTransform(path, X_ParentPath);
}

void Meshcat::Delete(std::string_view path) { publisher_->Delete(path); }

void Meshcat::SetProperty(std::string_view path, std::string property,
                          bool value) {
  publisher_->SetProperty(path, std::move(property), value);
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          double value) {
  publisher_->SetProperty(path, std::move(property), value);
}

bool Meshcat::HasPath(std::string_view path) const {
  return publisher_->HasPath(path);
}

std::string Meshcat::GetPackedObject(std::string_view path) const {
  return publisher_->GetPackedObject(path);
}

std::string Meshcat::GetPackedTransform(std::string_view path) const {
  return publisher_->GetPackedTransform(path);
}

std::string Meshcat::GetPackedProperty(std::string_view path,
                                       std::string property) const {
  return publisher_->GetPackedProperty(path, std::move(property));
}

}  // namespace geometry
}  // namespace drake
