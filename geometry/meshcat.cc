#include "drake/geometry/meshcat.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

#include <App.h>
#include <common_robotics_utilities/base64_helpers.hpp>
#include <fmt/format.h>
#include <msgpack.hpp>
#include <uuid.h>

#include "drake/common/drake_throw.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/common/unused.h"
#include "drake/geometry/meshcat_types.h"

#ifdef BOOST_VERSION
# error Drake should be using the non-boost flavor of msgpack.
#endif

// Steal one function declaration from usockets/src/internal/internal.h.
extern "C" {
void us_internal_free_closed_sockets(struct us_loop_t*);
}

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
  static const drake::never_destroyed<std::string> meshcat_js(
      LoadResource("drake/geometry/meshcat.js"));
  static const drake::never_destroyed<std::string> stats_js(
      LoadResource("drake/geometry/stats.min.js"));
  static const drake::never_destroyed<std::string> msgpack_lite_js(
      LoadResource("drake/geometry/msgpack.min.js"));
  static const drake::never_destroyed<std::string> meshcat_ico(
      LoadResource("drake/geometry/meshcat.ico"));
  static const drake::never_destroyed<std::string> meshcat_html(
      LoadResource("drake/geometry/meshcat.html"));
  static const drake::never_destroyed<std::string> empty;
  if ((url_path == "/")
      || (url_path == "/index.html")
      || (url_path == "/meshcat.html")) {
    return meshcat_html.access();
  }
  if (url_path == "/meshcat.js") {
    return meshcat_js.access();
  }
  if (url_path == "/stats.min.js") {
    return stats_js.access();
  }
  if (url_path == "/msgpack.min.js") {
    return msgpack_lite_js.access();
  }
  if (url_path == "/favicon.ico") {
    return meshcat_ico.access();
  }
  drake::log()->warn("Ignoring Meshcat http request for {}", url_path);
  return empty.access();
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

// Encode the meshcat command into a Javascript fetch() command.  The particular
// syntax using `fetch()` was replicated from the corresponding functionality in
// meshcat-python.
std::string CreateCommand(const std::string& data) {
  return fmt::format(R"""(
fetch("data:application/octet-binary;base64,{}")
    .then(res => res.arrayBuffer())
    .then(buffer => viewer.handle_command_bytearray(new Uint8Array(buffer)));
)""",
                     common_robotics_utilities::base64_helpers::Encode(
                         std::vector<uint8_t>(data.begin(), data.end())));
}

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

  // Returns a string which implements the entire tree directly in javascript.
  // This is intended for use in generating a "static html" of the scene.
  std::string CreateCommands() {
    std::string html;
    if (object_) {
      html += CreateCommand(*object_);
    }
    if (transform_) {
      html += CreateCommand(*transform_);
    }
    for (const auto& [property, msg] : properties_) {
      unused(property);
      html += CreateCommand(msg);
    }

    for (const auto& [name, child] : children_) {
      unused(name);
      html += child->CreateCommands();
    }
    return html;
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

  explicit MeshcatShapeReifier(uuids::uuid_random_generator* uuid_generator)
      : uuid_generator_(uuid_generator) {
    DRAKE_DEMAND(uuid_generator != nullptr);
  }

  ~MeshcatShapeReifier() = default;

  using ShapeReifier::ImplementGeometry;

  template <typename T>
  void ImplementMesh(const T& mesh, void* data) {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);

    // TODO(russt): Use file contents to generate the uuid, and avoid resending
    // meshes unless necessary.  Using the filename is tempting, but that leads
    // to problems when the file contents change on disk.

    const std::filesystem::path filename(mesh.filename());
    std::string format = filename.extension();
    format.erase(0, 1);  // remove the . from the extension
    std::ifstream input(mesh.filename(), std::ios::binary | std::ios::ate);
    if (!input.is_open()) {
      drake::log()->warn("Meshcat: Could not open mesh filename {}",
                         mesh.filename());
      return;
    }

    // We simply dump the binary contents of the file into the data field of the
    // message.  The javascript meshcat takes care of the rest.
    const int obj_size = input.tellg();
    input.seekg(0, std::ios::beg);
    std::string mesh_data;
    mesh_data.reserve(obj_size);
    mesh_data.assign(std::istreambuf_iterator<char>(input),
                     std::istreambuf_iterator<char>());

    // TODO(russt): MeshCat.jl/src/mesh_files.jl loads dae with textures, also.

    // TODO(russt): Make this mtllib parsing more robust (right now commented
    // mtllib lines will match, too, etc).
    size_t mtllib_pos;
    if (format == "obj" &&
        (mtllib_pos = mesh_data.find("mtllib ")) != std::string::npos) {
      mtllib_pos += 7;  // Advance to after the actual "mtllib " string.
      std::string mtllib_string =
          mesh_data.substr(mtllib_pos, mesh_data.find('\n', mtllib_pos));
      std::smatch matches;
      std::regex_search(mtllib_string, matches, std::regex("\\s*([^\\s]+)"));
      // Note: We do a minimal parsing manually here.  tinyobj does too much
      // work (actually loading all of the content) and also does not give
      // access to the intermediate data that we need to pass to meshcat, like
      // the resource names in the mtl file.  This is also the approach taken
      // in MeshCat.jl/src/mesh_files.jl.

      auto& meshfile_object =
          lumped.object.emplace<internal::MeshFileObjectData>();
      meshfile_object.uuid = uuids::to_string((*uuid_generator_)());
      meshfile_object.format = std::move(format);
      meshfile_object.data = std::move(mesh_data);

      std::string mtllib = matches.str(1);

      // Use filename path as the base directory for textures.
      const std::filesystem::path basedir = filename.parent_path();

      // Read .mtl file into geometry.mtl_library.
      std::ifstream mtl_stream(basedir / mtllib, std::ios::ate);
      if (mtl_stream.is_open()) {
        int mtl_size = mtl_stream.tellg();
        mtl_stream.seekg(0, std::ios::beg);
        meshfile_object.mtl_library.reserve(mtl_size);
        meshfile_object.mtl_library.assign(
            std::istreambuf_iterator<char>(mtl_stream),
            std::istreambuf_iterator<char>());

        // Scan .mtl file for map_ lines.  For each, load the file and add
        // the contents to geometry.resources.
        // The syntax (http://paulbourke.net/dataformats/mtl/) is e.g.
        //   map_Ka -options args filename
        // Here we ignore the options and only extract the filename (by
        // extracting the last word before the end of line/string).
        //  - "map_.+" matches the map_ plus any options,
        //  - "\s" matches one whitespace (before the filename),
        //  - "[^\s]+" matches the filename, and
        //  - "[$\r\n]" matches the end of string or end of line.
        // TODO(russt): This parsing could still be more robust.
        std::regex map_regex(R"""(map_.+\s([^\s]+)[$\r\n])""");
        for (std::sregex_iterator iter(meshfile_object.mtl_library.begin(),
                                       meshfile_object.mtl_library.end(),
                                       map_regex);
             iter != std::sregex_iterator(); ++iter) {
          std::string map = iter->str(1);
          std::ifstream map_stream(basedir / map,
                                   std::ios::binary | std::ios::ate);
          if (map_stream.is_open()) {
            int map_size = map_stream.tellg();
            map_stream.seekg(0, std::ios::beg);
            std::vector<uint8_t> map_data;
            map_data.reserve(map_size);
            map_data.assign(std::istreambuf_iterator<char>(map_stream),
                            std::istreambuf_iterator<char>());
            meshfile_object.resources.try_emplace(
                map, std::string("data:image/png;base64,") +
                          common_robotics_utilities::base64_helpers::Encode(
                              map_data));
          } else {
            drake::log()->warn(
                "Meshcat: Failed to load texture. {} references {}, but "
                "Meshcat could not open filename {}",
                basedir / mtllib, map, basedir / map);
          }
        }
      } else {
        drake::log()->warn(
            "Meshcat: Failed to load texture. {} references {}, but Meshcat "
            "could not open filename {}",
            mesh.filename(), mtllib, basedir / mtllib);
      }
      Eigen::Map<Eigen::Matrix4d> matrix(meshfile_object.matrix);
      matrix(0, 0) = mesh.scale();
      matrix(1, 1) = mesh.scale();
      matrix(2, 2) = mesh.scale();
    } else {  // not obj or no mtllib.
      auto geometry = std::make_unique<internal::MeshFileGeometryData>();
      geometry->uuid = uuids::to_string((*uuid_generator_)());
      geometry->format = std::move(format);
      geometry->data = std::move(mesh_data);
      lumped.geometry = std::move(geometry);

      auto& meshcat_mesh = lumped.object.emplace<internal::MeshData>();
      Eigen::Map<Eigen::Matrix4d> matrix(meshcat_mesh.matrix);
      matrix(0, 0) = mesh.scale();
      matrix(1, 1) = mesh.scale();
      matrix(2, 2) = mesh.scale();
    }
    }

  void ImplementGeometry(const Box& box, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    lumped.object = internal::MeshData();

    auto geometry = std::make_unique<internal::BoxGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->width = box.width();
    // Three.js uses height for the y axis; Drake uses depth.
    geometry->height = box.depth();
    geometry->depth = box.height();
    lumped.geometry = std::move(geometry);
  }

  void ImplementGeometry(const Capsule& capsule, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::CapsuleGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->radius = capsule.radius();
    geometry->length = capsule.length();
    lumped.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0))
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Convex& mesh, void* data) override {
    ImplementMesh(mesh, data);
  }

  void ImplementGeometry(const Cylinder& cylinder, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::CylinderGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->radiusBottom = cylinder.radius();
    geometry->radiusTop = cylinder.radius();
    geometry->height = cylinder.length();
    lumped.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0))
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Ellipsoid& ellipsoid, void* data) override {
    // Implemented as a Sphere stretched by a diagonal transform.
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::SphereGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->radius = 1;
    lumped.geometry = std::move(geometry);

    Eigen::Map<Eigen::Matrix4d> matrix(mesh.matrix);
    matrix(0, 0) = ellipsoid.a();
    matrix(1, 1) = ellipsoid.b();
    matrix(2, 2) = ellipsoid.c();
  }

  void ImplementGeometry(const HalfSpace&, void*) override {
    // TODO(russt): Use PlaneGeometry with fields width, height,
    // widthSegments, heightSegments
    drake::log()->warn("Meshcat does not display HalfSpace geometry (yet).");
  }

  void ImplementGeometry(const Mesh& mesh, void* data) override {
    ImplementMesh(mesh, data);
  }

  void ImplementGeometry(const MeshcatCone& cone, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::CylinderGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->radiusBottom = 0;
    geometry->radiusTop = 1.0;
    geometry->height = cone.height();
    lumped.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y and are centered at the
    // origin.  A cone is just a cylinder with radiusBottom=0.  So we transform
    // here, in addition to scaling to support non-uniform principle axes.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        Eigen::Vector4d{cone.a(), cone.b(), 1.0, 1.0}.asDiagonal() *
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0),
                        Eigen::Vector3d{0, 0, cone.height() / 2.0})
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Sphere& sphere, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    lumped.object = internal::MeshData();

    auto geometry = std::make_unique<internal::SphereGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->radius = sphere.radius();
    lumped.geometry = std::move(geometry);
  }

 private:
  uuids::uuid_random_generator* const uuid_generator_{};
};

int ToMeshcatColor(const Rgba& rgba) {
  // Note: The returned color discards the alpha value, which is handled
  // separately (e.g. by the opacity field in the material properties).
  return (static_cast<int>(255 * rgba.r()) << 16) +
         (static_cast<int>(255 * rgba.g()) << 8) +
         static_cast<int>(255 * rgba.b());
}

}  // namespace

class Meshcat::Impl {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Impl);

  // Some implementation notes for this Impl constructor:
  //
  // It must not call any nontrivial class methods. In general self-calls from
  // a constructor should always be treated with caution, but it's especially
  // important in this case because of the complicated threading and state
  // invariants that we need to maintain.
  //
  // It launches the websocket thread and waits for the thread to reply that
  // either the application started listning successfully, or else failed.
  //
  // If the websocket thread failed to bind to a port, then this constructor
  // will first join the websocket thread and then throw an exception; the
  // destructor will not be run.
  //
  // Otherwise, upon success the postconditions are that both:
  //   loop_ is non-null; and
  //   mode_ is either kFinished (the typical case) or possibly kStopping (in
  //     the unusual case where websocket thread faulted soon after starting).
  explicit Impl(const MeshcatParams& params)
      : prefix_("/drake"),
        main_thread_id_(std::this_thread::get_id()),
        params_(params) {
    DRAKE_THROW_UNLESS(params.port.value_or(7000) >= 1024);

    // Sanity-check the pattern, by passing it (along with dummy host and port
    // values) through to fmt to allow any fmt-specific exception to percolate.
    // Then, confirm that the user's pattern started with a valid protocol.
    const std::string url = fmt::format(
        fmt_runtime(params.web_url_pattern),
        fmt::arg("host", "foo"), fmt::arg("port", 1));
    if (url.substr(0, 4) != "http") {
      throw std::logic_error("The web_url_pattern must be http:// or https://");
    }

    // Fetch the index once to be sure that we preload the content.
    GetUrlContent("/");

    std::promise<std::tuple<int, bool>> app_promise;
    std::future<std::tuple<int, bool>> app_future =
        app_promise.get_future();
    websocket_thread_ = std::thread(
        &Impl::WrappedWebSocketMain, this, std::move(app_promise),
        params.host, params.port);
    bool connected;
    std::tie(port_, connected) = app_future.get();

    if (!connected) {
      mode_.store(kFinished);
      websocket_thread_.join();
      throw std::runtime_error("Meshcat failed to open a websocket port.");
    }
  }

  ~Impl() {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    // Ensure that the App::run loop stops, in case it hasn't already done so.
    Defer([this]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      Shutdown();
    });

    // Tell the websocket thread that we'll never call Defer() again,
    // and then wait for it to exit.
    mode_.store(kFinished);
    websocket_thread_.join();
  }

  // Throws an exception if the websocket thread has died.
  // This function is a file-internal helper, not public in the PIMPL.
  //
  // This should called from every public function of the outer class (other
  // than the destructor) before doing any other real work, so that we can pass
  // along error conditions from the websocket thread back onto the main thread.
  //
  // Don't fall into a TOCTOU trap here -- just because this function returned
  // successfully does *not* mean that the websocket thread is still running; it
  // might have crashed immediately after this check. Calling code should not
  // presume that success here means that additional calls into the websocket
  // will continue to succeed.
  void ThrowIfWebsocketThreadExited() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    // N.B. Refer to the comments on the `mode_` and `loop_` class member
    // fields to help understand what's happening here.
    if (mode_.load() != kNominal) {
      mode_.store(kFinished);
      throw std::runtime_error(
          "Meshcat's internal websocket thread exited unexpectedly");
    }
  }

  // This function is public via the PIMPL.
  std::string web_url() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    const std::string& host = params_.host;
    const bool is_localhost = host.empty() || host == "*";
    const std::string display_host = is_localhost ? "localhost" : host;
    return fmt::format(
        fmt_runtime(params_.web_url_pattern),
        fmt::arg("host", display_host),
        fmt::arg("port", port_));
  }

  // This function is public via the PIMPL.
  int port() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    return port_;
  }

  // This function is public via the PIMPL.
  void SetRealtimeRate(double rate) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    internal::RealtimeRateData data;
    data.rate = rate;
    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
    });
  }

  // This function is public via the PIMPL.
  std::string ws_url() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    const std::string http_url = web_url();
    DRAKE_DEMAND(http_url.substr(0, 4) == "http");
    return "ws" + http_url.substr(4);
  }

  // This function is public via the PIMPL.
  int GetNumActiveConnections() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    return num_websockets_.load();
  }

  // This function is public via the PIMPL.
  void Flush() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    // We simply loop until the backpressure is zero.  In each iteration, if
    // the connections have any backpressure, then we sleep the main thread to
    // let the websocket thread drain.
    //
    // Note: The following attempts to avoid the explicit sleep failed:
    // - loop_->defer(callback) does not drain automatically between executing
    //   the deferred callbacks.
    // - calling app_->topicTree->drain() or ws->send("") did not actually
    //   force any drainage.
    //
    // It *is* possible to monitor the drainage by registering the
    // behavior.drain callback on the websocket connection, but we avoid
    // explicitly locking the main thread to wait for the drainage in order to
    // keep the thread logic simpler.
    int main_backpressure;

    // Set a very conservative iteration limit; since we sleep for .1 seconds
    // on each iteration, this corresponds to (approximately) 10 minutes.
    const int kIterationLimit{6000};
    int iteration = 0;

    do {
      std::promise<int> p;
      std::future<int> f = p.get_future();
      Defer([this, p = std::move(p)]() mutable {
        DRAKE_DEMAND(IsThread(websocket_thread_id_));
        int websocket_backpressure = 0;
        for (WebSocket* ws : websockets_) {
          websocket_backpressure += ws->getBufferedAmount();
        }
        p.set_value(websocket_backpressure);
      });
      main_backpressure = f.get();
      if (main_backpressure > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      ++iteration;
    } while (main_backpressure > 0 && iteration < kIterationLimit);

    if (main_backpressure > 0) {
      drake::log()->warn(
          "Meshcat::Flush() reached an iteration limit before the buffer could "
          "be completely flushed.");
    }
  }

  // This function is public via the PIMPL.
  void SetObject(std::string_view path, const Shape& shape, const Rgba& rgba) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetObjectData data;
    data.path = FullPath(path);

    // TODO(russt): This current meshcat set_object interface couples geometry,
    // material, and object for convenience, but we might consider decoupling
    // them again for efficiency. We don't want to send meshes over the network
    // (which could be from the cloud to a local browser) more than necessary.

    MeshcatShapeReifier reifier(&uuid_generator);
    shape.Reify(&reifier, &data.object);

    if (std::holds_alternative<std::monostate>(data.object.object)) {
      // Then this shape is not supported, and I should not send the message,
      // nor add the object to the tree.
      return;
    }
    if (std::holds_alternative<internal::MeshData>(data.object.object)) {
      auto& meshfile_object = std::get<internal::MeshData>(data.object.object);
      DRAKE_DEMAND(data.object.geometry != nullptr);
      meshfile_object.geometry = data.object.geometry->uuid;

      auto material = std::make_unique<internal::MaterialData>();
      material->uuid = uuids::to_string(uuid_generator());
      material->type = "MeshPhongMaterial";
      material->color = ToMeshcatColor(rgba);
      // TODO(russt): Most values are taken verbatim from meshcat-python.
      material->reflectivity = 0.5;
      material->side = internal::kDoubleSide;
      // From meshcat-python: Three.js allows a material to have an opacity
      // which is != 1, but to still be non - transparent, in which case the
      // opacity only serves to desaturate the material's color. That's a
      // pretty odd combination of things to want, so by default we just use
      // the opacity value to decide whether to set transparent to True or
      // False.
      material->transparent = (rgba.a() != 1.0);
      material->opacity = rgba.a();
      material->linewidth = 1.0;
      material->wireframe = false;
      material->wireframeLineWidth = 1.0;

      meshfile_object.uuid = uuids::to_string(uuid_generator());
      meshfile_object.material = material->uuid;
      data.object.material = std::move(material);
    }

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      // TODO(russt): Consider using msgpack::sbuffer instead of stringstream
      // (here and throughout) to avoid this copy.
      // https://github.com/redboltz/msgpack-c/wiki/v2_0_cpp_packer
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetObject(std::string_view path, const perception::PointCloud& cloud,
                 double point_size, const Rgba& rgba) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuids::to_string(uuid_generator());
    geometry->position = cloud.xyzs();
    if (cloud.has_rgbs()) {
      geometry->color = cloud.rgbs().cast<float>()/255.0;
    }
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuids::to_string(uuid_generator());
    material->type = "PointsMaterial";
    material->color = ToMeshcatColor(rgba);
    material->transparent = (rgba.a() != 1.0);
    material->opacity = rgba.a();
    material->size = point_size;
    material->vertexColors = cloud.has_rgbs();
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuids::to_string(uuid_generator());
    mesh.type = "Points";
    mesh.geometry = data.object.geometry->uuid;
    mesh.material = data.object.material->uuid;
    data.object.object = std::move(mesh);

    Defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetObject(std::string_view path, const TriangleSurfaceMesh<double>& mesh,
                 const Rgba& rgba, bool wireframe,
                 double wireframe_line_width) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    Eigen::Matrix3Xd vertices(3, mesh.num_vertices());
    for (int i = 0; i < mesh.num_vertices(); ++i) {
      vertices.col(i) = mesh.vertex(i);
    }
    Eigen::Matrix3Xi faces(3, mesh.num_triangles());
    for (int i = 0; i < mesh.num_triangles(); ++i) {
      const auto& e = mesh.element(i);
      for (int j = 0; j < 3; ++j) {
        faces(j, i) = e.vertex(j);
      }
    }
    SetTriangleMesh(path, vertices, faces, rgba, wireframe,
                    wireframe_line_width);
  }

  // This function is public via the PIMPL.
  void SetLine(std::string_view path,
               const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
               double line_width, const Rgba& rgba) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    const bool kLineSegments = false;
    SetLineImpl(path, vertices, line_width, rgba, kLineSegments);
  }

  // This function is public via the PIMPL.
  void SetLineSegments(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& start,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& end,
                       double line_width, const Rgba& rgba) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_THROW_UNLESS(start.cols() == end.cols());
    // The LineSegments loader in three.js take the same data structure as Line,
    // but takes every consecutive pair of vertices as a (start, end).
    Eigen::Matrix<double, 6, Eigen::Dynamic> vstack(6, start.cols());
    vstack << start, end;
    Eigen::Map<Eigen::Matrix3Xd> vertices(vstack.data(), 3, 2*start.cols());
    const bool kLineSegments = true;
    SetLineImpl(path, vertices, line_width, rgba, kLineSegments);
  }

  // This function is internal to the PIMPL, used to implement the prior two
  // functions (SetLine and SetLineSegments).
  void SetLineImpl(std::string_view path,
                   const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                   double line_width, const Rgba& rgba, bool line_segments) {
    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuids::to_string(uuid_generator());
    geometry->position = vertices.cast<float>();
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuids::to_string(uuid_generator());
    material->type = "LineBasicMaterial";
    material->color = ToMeshcatColor(rgba);
    material->linewidth = line_width;
    material->vertexColors = false;
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuids::to_string(uuid_generator());
    mesh.type = line_segments ? "LineSegments" : "Line";
    mesh.geometry = data.object.geometry->uuid;
    mesh.material = data.object.material->uuid;
    data.object.object = std::move(mesh);

    Defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetTriangleMesh(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                       const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                       const Rgba& rgba, bool wireframe,
                       double wireframe_line_width) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuids::to_string(uuid_generator());
    geometry->position = vertices.cast<float>();
    geometry->faces = faces.cast<uint32_t>();
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuids::to_string(uuid_generator());
    material->type = "MeshPhongMaterial";
    material->color = ToMeshcatColor(rgba);
    material->transparent = (rgba.a() != 1.0);
    material->opacity = rgba.a();
    material->wireframe = wireframe;
    material->wireframeLineWidth = wireframe_line_width;
    material->vertexColors = false;
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuids::to_string(uuid_generator());
    mesh.type = "Mesh";
    mesh.geometry = data.object.geometry->uuid;
    mesh.material = data.object.material->uuid;
    data.object.object = std::move(mesh);

    Defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetTriangleColorMesh(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                       const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& colors,
                       bool wireframe,
                       double wireframe_line_width) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuids::to_string(uuid_generator());
    geometry->position = vertices.cast<float>();
    geometry->faces = faces.cast<uint32_t>();
    geometry->color = colors.cast<float>();
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuids::to_string(uuid_generator());
    material->type = "MeshPhongMaterial";
    material->transparent = false;
    material->opacity = 1.0;
    material->wireframe = wireframe;
    material->wireframeLineWidth = wireframe_line_width;
    material->vertexColors = true;
    material->side = internal::kDoubleSide;
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuids::to_string(uuid_generator());
    mesh.type = "Mesh";
    mesh.geometry = data.object.geometry->uuid;
    mesh.material = data.object.material->uuid;
    data.object.object = std::move(mesh);

    Defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  template <typename CameraData>
  void SetCamera(CameraData camera, std::string path) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetCameraData<CameraData> data;
    data.path = std::move(path);
    data.object.object = std::move(camera);

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetTransform(std::string_view path,
                    const RigidTransformd& X_ParentPath) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    SetTransform(path, X_ParentPath.GetAsMatrix4());
  }

  // This function is public via the PIMPL.
  void SetTransform(std::string_view path,
                    const Eigen::Ref<const Eigen::Matrix4d>& matrix) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetTransformData data;
    data.path = FullPath(path);
    Eigen::Map<Eigen::Matrix4d>(data.matrix) = matrix;

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.transform() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void Delete(std::string_view path) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::DeleteData data;
    data.path = FullPath(path);

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
      scene_tree_root_.Delete(data.path);
    });
  }

  // This function is public via the PIMPL, via overloads for a specific set of
  // template types (not all possible T's).
  template <typename T>
  void SetProperty(std::string_view path, std::string property,
                   const T& value) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetPropertyData<T> data;
    data.path = FullPath(path);
    data.property = std::move(property);
    data.value = value;

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.properties()[data.property] = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetAnimation(const MeshcatAnimation& animation) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::stringstream message_stream;
    // We pack this message in-place (rather than using structs to organize
    // the packing) for a few reasons:
    //  1) we want to avoid copying the big data nested structure,
    //  2) this message type would require a nasty hairball of structs, and
    //  3) the nested structures have paths inside that must be modified with
    //     FullPath().
    msgpack::packer o(message_stream);
    // The details of this message have been extracted primarily from
    // meshcat/test/animation.html and
    // meshcat-python/src/meshcat/animation.py.
    o.pack_map(3);
    o.pack("type");
    o.pack("set_animation");
    o.pack("animations");
    {
      o.pack_array(animation.path_tracks_.size());
      for (const auto& path_track : animation.path_tracks_) {
        o.pack_map(2);
        o.pack("path");
        // TODO(russt): Handle the case where the FullPaths are not unique.
        o.pack(FullPath(path_track.first));
        o.pack("clip");
        {
          o.pack_map(3);
          o.pack("fps");
          o.pack(animation.frames_per_second());
          o.pack("name");
          o.pack("default");
          o.pack("tracks");
          {
            o.pack_array(path_track.second.size());
            for (const auto& property_track : path_track.second) {
              o.pack_map(3);
              o.pack("name");
              o.pack("." + property_track.first);
              o.pack("type");
              o.pack(property_track.second.js_type);
              o.pack("keys");
              std::visit(
                  [&o](const auto& track) {
                    using T = std::decay_t<decltype(track)>;
                    if constexpr (!std::is_same_v<T, std::monostate>) {
                      o.pack_array(track.size());
                      for (const auto& key : track) {
                        o.pack_map(2);
                        o.pack("time");
                        o.pack(key.first);
                        o.pack("value");
                        o.pack(key.second);
                      }
                    }
                  },
                  property_track.second.track);
            }
          }
        }
      }
    }
    o.pack("options");
    {
      o.pack_map(4);
      o.pack("play");
      o.pack(animation.autoplay());
      o.pack("loopMode");
      o.pack(animation.loop_mode());
      o.pack("repetitions");
      o.pack(animation.repetitions());
      o.pack("clampWhenFinished");
      o.pack(animation.clamp_when_finished());
    }

    Defer(
        [this, message = message_stream.str()]() {
          DRAKE_DEMAND(IsThread(websocket_thread_id_));
          DRAKE_DEMAND(app_ != nullptr);
          app_->publish("all", message, uWS::OpCode::BINARY, false);
          animation_ = std::move(message);
        });
  }

  // This function is public via the PIMPL.
  void Set2dRenderMode(const math::RigidTransformd& X_WC, double xmin,
                      double xmax, double ymin, double ymax) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    // Set orthographic camera.
    OrthographicCamera camera;
    camera.left = xmin;
    camera.right = xmax;
    camera.bottom = ymin;
    camera.top = ymax;
    SetCamera(camera, "/Cameras/default/rotated");

    SetTransform("/Cameras/default", X_WC);
    // Lock orbit controls.
    SetProperty("/Cameras/default/rotated/<object>", "position",
                std::vector<double>{0.0, 0.0, 0.0});

    SetProperty("/Background", "visible", false);
    SetProperty("/Grid", "visible", false);
    SetProperty("/Axes", "visible", false);
  }

  // This function is public via the PIMPL.
  void ResetRenderMode() {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    PerspectiveCamera camera;
    SetCamera(camera, "/Cameras/default/rotated");
    SetTransform("/Cameras/default", math::RigidTransformd());
    // Lock orbit controls.
    SetProperty("/Cameras/default/rotated/<object>", "position",
                std::vector<double>{0.0, 1.0, 3.0});
    SetProperty("/Background", "visible", true);
    SetProperty("/Grid", "visible", true);
    SetProperty("/Axes", "visible", true);
  }

  // This function is public via the PIMPL.
  void AddButton(std::string name, std::string keycode) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetButtonControl data;
    data.name = std::move(name);
    data.callback = fmt::format(R"""(
() => this.connection.send(msgpack.encode({{
  'type': 'button',
  'name': '{}'
}})))""", data.name);
    data.keycode1 = std::move(keycode);

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      if (sliders_.find(data.name) != sliders_.end()) {
        throw std::logic_error(
            fmt::format("Meshcat already has a slider named {}.", data.name));
      }
      auto iter = buttons_.find(data.name);
      if (iter == buttons_.end()) {
        controls_.emplace_back(data.name);
      } else {
        iter->second.num_clicks = 0;
        if (iter->second.keycode1.empty()) {
          if (data.keycode1.empty()) {
            // No need to publish to meshcat.
            return;
          }  // else fall through.
        } else if (iter->second.keycode1 != data.keycode1) {
          throw std::logic_error(fmt::format(
              "Meshcat already has a button named `{}`, but the previously "
              "assigned keycode `{}` does not match the current keycode `{}`. "
              "To re-assign the keycode, you must first delete the button.",
              data.name, iter->second.keycode1, data.keycode1));
        }
      }
      buttons_[data.name] = data;
      DRAKE_DEMAND(controls_.size() == (buttons_.size() + sliders_.size()));
    }

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  // This function is public via the PIMPL.
  int GetButtonClicks(std::string_view name) {
    std::lock_guard<std::mutex> lock(controls_mutex_);
    auto iter = buttons_.find(name);
    if (iter == buttons_.end()) {
      throw std::logic_error(
          fmt::format("Meshcat does not have any button named {}.", name));
    }
    return iter->second.num_clicks;
  }

  // This function is public via the PIMPL.
  void DeleteButton(std::string name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::DeleteControl data;
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = buttons_.find(name);
      if (iter == buttons_.end()) {
        throw std::logic_error(
            fmt::format("Meshcat does not have any button named {}.", name));
      }
      buttons_.erase(iter);
      auto c_iter = std::find(controls_.begin(), controls_.end(), name);
      DRAKE_DEMAND(c_iter != controls_.end());
      controls_.erase(c_iter);
      data.name = std::move(name);
      DRAKE_DEMAND(controls_.size() == (buttons_.size() + sliders_.size()));
    }

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  // This function is public via the PIMPL.
  void AddSlider(std::string name, double min, double max, double step,
                 double value, std::string decrement_keycode,
                 std::string increment_keycode) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetSliderControl data;
    data.name = std::move(name);
    data.callback = fmt::format(R"""(
(value) => this.connection.send(msgpack.encode({{
  'type': 'slider',
  'name': '{}',
  'value': value
}})))""", data.name);
    data.min = min;
    data.max = max;
    data.step = step;
    // Match setValue in NumberController.js from dat.GUI.
    // https://github.com/dataarts/dat.gui/blob/f720c729deca5d5c79da8464f8a05500d38b140c/src/dat/controllers/NumberController.js#L62
    value = std::max(value, min);
    value = std::min(value, max);
    value = std::round(value / step) * step;
    data.value = value;
    data.keycode1 = std::move(decrement_keycode);
    data.keycode2 = std::move(increment_keycode);

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      if (buttons_.find(data.name) != buttons_.end()) {
        throw std::logic_error(
            fmt::format("Meshcat already has a button named {}.", data.name));
      }
      if (sliders_.find(data.name) != sliders_.end()) {
        throw std::logic_error(
            fmt::format("Meshcat already has a slider named {}.", data.name));
      }
      controls_.emplace_back(data.name);
      sliders_[data.name] = data;
      DRAKE_DEMAND(controls_.size() == (buttons_.size() + sliders_.size()));
    }

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  // This function is public via the PIMPL.
  void SetSliderValue(std::string name, double value) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = sliders_.find(name);
      if (iter == sliders_.end()) {
        throw std::logic_error(
            fmt::format("Meshcat does not have any slider named {}.", name));
      }
      internal::SetSliderControl& s = iter->second;
      // Match setValue in NumberController.js from dat.GUI.
      value = std::max(value, s.min);
      value = std::min(value, s.max);
      value = std::round(value/s.step)*s.step;
      s.value = value;
    }

    internal::SetSliderValue data;
    data.name = std::move(name);
    data.value = value;

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  // This function is public via the PIMPL.
  double GetSliderValue(std::string_view name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::lock_guard<std::mutex> lock(controls_mutex_);
    auto iter = sliders_.find(name);
    if (iter == sliders_.end()) {
      throw std::logic_error(
          fmt::format("Meshcat does not have any slider named {}.", name));
    }
    return iter->second.value;
  }

  // This function is public via the PIMPL.
  void DeleteSlider(std::string name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::DeleteControl data;
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = sliders_.find(name);
      if (iter == sliders_.end()) {
        throw std::logic_error(
            fmt::format("Meshcat does not have any slider named {}.", name));
      }
      sliders_.erase(iter);
      auto c_iter = std::find(controls_.begin(), controls_.end(), name);
      DRAKE_DEMAND(c_iter != controls_.end());
      controls_.erase(c_iter);
      data.name = std::move(name);
      DRAKE_DEMAND(controls_.size() == (buttons_.size() + sliders_.size()));
    }

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  // This function is public via the PIMPL.
  void DeleteAddedControls() {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    // We copy the data structures so that the main thread can iterate through
    // them without acquiring an additional lock.
    std::map<std::string, internal::SetButtonControl, std::less<>> buttons{};
    std::map<std::string, internal::SetSliderControl, std::less<>> sliders{};
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      buttons = buttons_;
      sliders = sliders_;
    }
    for (auto iter = buttons.begin(); iter != buttons.end(); ++iter) {
      DeleteButton(iter->first);
    }
    for (auto iter = sliders.begin(); iter != sliders.end(); ++iter) {
      DeleteSlider(iter->first);
    }
  }

  // This function is public via the PIMPL.
  std::string StaticHtml() {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    std::string html = GetUrlContent("/");

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    Defer([this, p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      std::string commands = scene_tree_root_.CreateCommands();
      if (!animation_.empty()) {
        commands += CreateCommand(animation_);
      }
      p.set_value(std::move(commands));
    });

    // Replace the javascript code in the original html file which connects via
    // websockets with the static javascript commands.
    std::regex block_re(
        "<!-- CONNECTION BLOCK BEGIN [^]+ CONNECTION BLOCK END -->\n");
    html = std::regex_replace(html, block_re, f.get());

    // Insert the javascript directly into the html.
    std::vector<std::pair<std::string, std::string>> js_paths{
        {" src=\"meshcat.js\"", "/meshcat.js"},
        {" src=\"stats.min.js\"", "/stats.min.js"},
        {" src=\"msgpack.min.js\"", "/msgpack.min.js"},
    };

    for (const auto& [src_link, url] : js_paths) {
      const size_t js_pos = html.find(src_link);
      DRAKE_DEMAND(js_pos != std::string::npos);
      html.erase(js_pos, src_link.size());
      html.insert(js_pos+1, GetUrlContent(url));
    }

    return html;
  }

  // This function is public via the PIMPL.
  bool HasPath(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::promise<bool> p;
    std::future<bool> f = p.get_future();
    Defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      p.set_value(scene_tree_root_.Find(path) != nullptr);
    });
    return f.get();
  }

  // This function is public via the PIMPL.
  std::string GetPackedObject(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    Defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      const SceneTreeElement* e = scene_tree_root_.Find(path);
      if (!e || !e->object()) {
        p.set_value("");
      } else {
        p.set_value(*e->object());
      }
    });
    return f.get();
  }

  // This function is public via the PIMPL.
  std::string GetPackedTransform(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    Defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      const SceneTreeElement* e = scene_tree_root_.Find(path);
      if (!e || !e->transform()) {
        p.set_value("");
      } else {
        p.set_value(*e->transform());
      }
    });
    return f.get();
  }

  // This function is public via the PIMPL.
  std::string GetPackedProperty(std::string_view path,
                                std::string property) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    Defer([this, path = FullPath(path), property = std::move(property),
                  p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
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

  void InjectWebsocketThreadFault(int fault_number) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(fault_number >= 0);
    DRAKE_DEMAND(fault_number <= kMaxFaultNumber);
    log()->warn("InjectWebsocketThreadFault({}) was called", fault_number);
    switch (fault_number) {
      case 0: {
        Defer([this]() {
          DRAKE_DEMAND(IsThread(websocket_thread_id_));
          // Closing the listen socket will cause the app.run() loop to exit.
          us_listen_socket_close(0, listen_socket_);
          listen_socket_ = nullptr;
        });
        return;
      }
      case 1: {
        Defer([this]() {
          DRAKE_DEMAND(IsThread(websocket_thread_id_));
          throw std::runtime_error("InjectWebsocketThreadFault during defer");
        });
        return;
      }
      case 2: {
        inject_open_fault_.store(true);
        return;
      }
      case 3: {
        inject_message_fault_.store(true);
        return;
      }
      static_assert(kMaxFaultNumber == 3);
    }
    DRAKE_UNREACHABLE();
  }

 private:
  bool IsThread(std::thread::id thread_id) const {
    return (std::this_thread::get_id() == thread_id);
  }

  // This is the entry point for our websocket thread. Its only job is as a
  // last-resort exception catcher so that we'll always log it and never call
  // std::terminate (an exception leaking from std::thread always terminates).
  //
  // Our design goal is that no exception can ever reach this function anyway
  // (it should be caught by a more local try-catch block) but in case we've
  // missed one of those, we want to be sure to log it here.
  //
  // Catching exceptions is generally prohibited by Drake's style guide, but
  // in this case the std::terminate fall-through is too painful to live with,
  // and we end up re-throwing an exception on the main thread eventually.
  //
  // N.B. Our arguments must not be pass-by-reference because this function is
  // called from a new thread!
  void WrappedWebSocketMain(
      std::promise<std::tuple<int, bool>> app_promise,
      std::string host, std::optional<int> desired_port) {
    try {
      WebSocketMain(std::move(app_promise), host, desired_port);
    } catch (const std::exception& e) {
      drake::log()->critical(
          "Meshcat's internal websocket thread crashed via an exception: {}",
          e.what());
    }
  }

  void WebSocketMain(
      std::promise<std::tuple<int, bool>> app_promise,
      const std::string& host, std::optional<int> desired_port) {
    websocket_thread_id_ = std::this_thread::get_id();
    ScopeExit guard([this]() {
      // N.B. Refer to the comments on the `mode_` and `loop_` class member
      // fields to help understand what's happening here.
      OperatingMode nominal = kNominal;
      mode_.compare_exchange_strong(nominal, kStopping);
      // We must not exit this thread (destroying the thread_local uWS::Loop)
      // until we know that the main thread is no longer adding more callbacks.
      // It signals that by setting mode_ to kFinshed.
      while (mode_.load() != kFinished) {
        // TODO(jwnimmer-tri) Use atomic::wait instead, once we have C++20.
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      loop_ = nullptr;
      // Given this scope guard, the post-condition upon return from
      // WebSocketMain is that mode_ is kFinished and loop_ is null.
    });
    loop_ = uWS::Loop::get();

    // This canonicalization of the bind_host is currently redundant with what
    // uWebSockets already implements, but we'll keep it here anyway, to defend
    // our code from potential implementation changes to uWebSockets.
    const std::string bind_host = (host == "*") ? "" : host;

    int port = desired_port ? *desired_port : 7000;
    const int kMaxPort = desired_port ? *desired_port : 7099;

    uWS::App::WebSocketBehavior<PerSocketData> behavior;
    // Set maxBackpressure = 0 so that uWS does *not* drop any messages due to
    // back pressure.
    behavior.maxBackpressure = 0;
    behavior.open = [this](WebSocket* ws) {
      // IsThread(websocket_thread_id_) is checked by the Handle... function.
      HandleSocketOpen(ws);
    };
    behavior.close = [this](WebSocket* ws, int, std::string_view message) {
      // IsThread(websocket_thread_id_) is checked by the Handle... function.
      unused(message);
      HandleSocketClose(ws);
    };
    behavior.message = [this](WebSocket* ws, std::string_view message,
                              uWS::OpCode op_code) {
      // IsThread(websocket_thread_id_) is checked by the Handle... function.
      unused(op_code);
      HandleMessage(ws, message);
    };

    uWS::App app =
        uWS::App()
            .get("/*",
                 [&](uWS::HttpResponse<kSsl>* res, uWS::HttpRequest* req) {
                   DRAKE_DEMAND(IsThread(websocket_thread_id_));
                   res->end(GetUrlContent(req->getUrl()));
                 })
            .ws<PerSocketData>("/*", std::move(behavior));
    app_ = &app;

    do {
      app.listen(
          bind_host, port, LIBUS_LISTEN_EXCLUSIVE_PORT,
          [this](us_listen_socket_t* socket) {
            DRAKE_DEMAND(IsThread(websocket_thread_id_));
            if (socket) {
              listen_socket_ = socket;
            }
          });
    } while (listen_socket_ == nullptr && port++ < kMaxPort);

    bool connected = listen_socket_ != nullptr;
    app_promise.set_value(std::make_tuple(port, connected));

    if (!connected) {
      return;
    }

    ScopeExit listen_guard([this]() {
      if (listen_socket_ != nullptr) {
        drake::log()->warn(
            "Meshcat's internal websocket is stopping via an exception");
        Shutdown();
        // Normally uWS will free all of its memory as part of App shutdown.
        // However, when exiting via exception it only places the socket memory
        // onto a close-list instead of freeing it. To avoid heap leaks, we'll
        // manually free the memory here using an internal helper function.
        // TODO(jwnimmer-tri) Probably uWS::LoopCleaner::~LoopCleaner should be
        // doing this? Submit a ticket with upstream to find the correct answer.
        us_internal_free_closed_sockets(
            reinterpret_cast<struct us_loop_t*>(uWS::Loop::get()));
      }
    });

    app.run();
  }

  // This function is a callback from a WebSocketBehavior.
  void HandleSocketOpen(WebSocket* ws) {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));
    drake::log()->debug(
        "Meshcat connection opened from {}",
        ws->getRemoteAddressAsText());
    websockets_.emplace(ws);
    const int new_count = ++num_websockets_;
    DRAKE_DEMAND(new_count >= 0);
    DRAKE_DEMAND(new_count == static_cast<int>(websockets_.size()));
    ws->subscribe("all");
    // Update this new connection with previously published data.
    scene_tree_root_.Send(ws);
    if (!animation_.empty()) {
      ws->send(animation_);
    }
    std::lock_guard<std::mutex> lock(controls_mutex_);
    for (const auto& c : controls_) {
      auto b_iter = buttons_.find(c);
      if (b_iter != buttons_.end()) {
        std::stringstream message_stream;
        msgpack::pack(message_stream, b_iter->second);
        ws->send(message_stream.str());
      } else {
        auto s_iter = sliders_.find(c);
        DRAKE_DEMAND(s_iter != sliders_.end());
        std::stringstream message_stream;
        msgpack::pack(message_stream, s_iter->second);
        ws->send(message_stream.str());
      }
    }

    // Tell client if the realtime rate plot should be hidden
    internal::ShowRealtimeRate realtime_rate_message;
    realtime_rate_message.show = params_.show_stats_plot;
    std::stringstream realtime_message_stream;
    msgpack::pack(realtime_message_stream, realtime_rate_message);
    ws->send(realtime_message_stream.str());

    if (inject_open_fault_.load()) {
      throw std::runtime_error(
          "InjectWebsocketThreadFault during socket open");
    }
  }

  // This function is a callback from a WebSocketBehavior.
  void HandleSocketClose(WebSocket* ws) {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));
    drake::log()->debug(
        "Meshcat connection closed from {}",
        ws->getRemoteAddressAsText());
    websockets_.erase(ws);
    const int new_count = --num_websockets_;
    DRAKE_DEMAND(new_count >= 0);
    DRAKE_DEMAND(new_count == static_cast<int>(websockets_.size()));
  }

  // This function is a callback from a WebSocketBehavior.
  void HandleMessage(WebSocket* ws, std::string_view message) {
    internal::UserInterfaceEvent data;
    try {
      msgpack::object_handle o_h =
          msgpack::unpack(message.data(), message.size());
      o_h.get().convert(data);
    } catch (const msgpack::type_error& e) {
      // Quietly ignore messages that don't match our expected message type.
      // This violates the style guide, but msgpack does not provide any other
      // mechanism for checking the message type.
      drake::log()->debug("Meshcat ignored an unparseable message");
      return;
    }
    std::lock_guard<std::mutex> lock(controls_mutex_);
    if (data.type == "button") {
      auto iter = buttons_.find(data.name);
      if (iter != buttons_.end()) {
        iter->second.num_clicks++;
      }
      return;
    }
    if (data.type == "slider" && data.value.has_value()) {
      auto iter = sliders_.find(data.name);
      if (iter != sliders_.end()) {
        iter->second.value = *data.value;
        if (websockets_.size() > 1) {
          // Send SetSliderValue message to all other websockets.
          internal::SetSliderValue set_slider;
          set_slider.name = std::move(data.name);
          set_slider.value = *data.value;
          std::stringstream message_stream;
          msgpack::pack(message_stream, set_slider);
          // ws->publish sends to all but not to ws.
          ws->publish("all", message_stream.str(), uWS::OpCode::BINARY);
        }
      }
      return;
    }
    drake::log()->warn("Meshcat ignored a '{}' event", data.type);
    if (inject_message_fault_.load()) {
      throw std::runtime_error(
          "InjectWebsocketThreadFault during message callback");
    }
  }

  // A functor object that we can post from the main thread into the websocket
  // thread.
  using Callback = uWS::MoveOnlyFunction<void()>;

  // This function is a private utility for use within this class.
  // It posts the given callback into the websocket thread, safely.
  // If the websocket thread is no longer operating, then this function will
  // destroy the callback, without ever invoking it.
  void Defer(Callback&& callback) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    if (mode_.load() == kNominal) {
      DRAKE_DEMAND(loop_ != nullptr);
      loop_->defer(std::move(callback));
    }
  }

  // This function is a private utility for use within this class. It closes all
  // sockets therefore will cause the uWS::App::run() function to return, and
  // therefore the worker thread will (eventually) exit. This should only be
  // called from two places: in the case of graceful shutdown as a deferred
  // event posted by the ~Impl destructor, or in the case of faulty shutdown
  // in the websocket thread's scope guard.
  void Shutdown() {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));
    drake::log()->debug("Meshcat Shutdown");

    // Stop accepting new connections.
    if (listen_socket_ != nullptr) {
      us_listen_socket_close(0, listen_socket_);
      listen_socket_ = nullptr;
    }

    // Close any existing connections. Calling ws->close() erases the WebSocket
    // from websockets_, so we need to advance the iterator beforehand (#15821).
    auto iter = websockets_.begin();
    while (iter != websockets_.end()) {
      WebSocket* ws = *iter++;
      ws->close();
    }
  }

  // This function is a private utility for use within this class.
  std::string FullPath(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
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

  // Both threads access controls_, guarded by controls_mutex_.
  std::mutex controls_mutex_;
  std::map<std::string, internal::SetButtonControl, std::less<>> buttons_{};
  std::map<std::string, internal::SetSliderControl, std::less<>> sliders_{};
  std::vector<std::string> controls_{};  // Names of buttons and sliders in the
                                         // order they were added.

  // These variables should only be accessed in the main thread, where "main
  // thread" is the thread in which this class was constructed.
  std::thread::id main_thread_id_{};
  const MeshcatParams params_;
  int port_{};
  std::mt19937 generator_{};

  // These variables should only be accessed in the websocket thread.
  std::thread::id websocket_thread_id_{};
  SceneTreeElement scene_tree_root_{};
  std::string animation_;
  uWS::App* app_{nullptr};
  us_listen_socket_t* listen_socket_{nullptr};
  std::set<WebSocket*> websockets_{};

  // This variable may be accessed from any thread, but should only be modified
  // in the websocket thread.
  std::atomic<int> num_websockets_{0};

  // The loop_ pointer is used to pass functors from the main thread into the
  // websocket worker thread, via loop_->defer(...). See the documentation of
  // uWebSockets for further details:
  // https://github.com/uNetworking/uWebSockets/blob/d94bf2c/misc/READMORE.md#threading
  //
  // We must be *extremely careful* with the lifecycle of the loop_ pointer.
  //
  // It's initialized to nullptr. Then, our constructor launches the websocket
  // thread and blocks on a future until the thread is ready. Before declaring
  // itself ready via the promise, the websocket thread sets loop_ to the
  // correct value. Therefore, when our constructor returns we can rest assured
  // that the loop_ contains a valid value.
  //
  // However, the *grave danger* here is that loop_ points to uWS::Loop::get,
  // which is a *thread_local static object*. When the thread finishes, the C++
  // runtime will destroy the thread_local object and our loop_ will become a
  // dangling pointer. Calling any function on it would lead to nasal demons.
  //
  // Therefore, we must prevent the websocket thread from exiting until it's
  // sure that the main thread will never again use the loop_. We implement
  // that using a scope guard within the websocket thread, where it monitors
  // the mode_ member. Only once mode_ is set to kFinished may the thread be
  // allowed to exit. We also encapsulate all main-thread access to loop_
  // within the Defer() helper function, to help maintain this invariant.
  //
  // In short, our invariant is that: loop_ is only guaranteed to be non-null
  // when mode_ is either kNominal or kStopping.
  uWS::Loop* loop_{nullptr};

  // The other half of maintaining the loop lifecycle invariant (as described
  // on the loop_ member above) is the mode_ enum.
  //
  // The mode_ enum is always set to one of exactly three values:
  // - kNominal
  // - kStopping
  // - kFinished
  //
  // It begins life set to Nominal. In this mode, calls to the Defer()
  // helper are passed along to the websocket thread's uWS event loop.
  //
  // Within the websocket thread, anytime the thread is about to exit (whether
  // through normal means or an exception), the scope guard takes over and
  // does an atomic compare-and-swap, demoting Nominal to Stopping. This is
  // the indication to the main thread that the websocket thread is shutting
  // down. Any *thereafter* calls to Defer() will destroy the callback functor
  // instead of posting it into the uWS loop.
  //
  // Note, however, that if a call to Defer() was partway through execution
  // (where it had done mode_.load(), but not yet called into the loop_),
  // then callbacks could still be placed into the uWS loop even after the
  // compare-and-swap had completed. This is still fine. They will not be run
  // (because the loop isn't running), but they will be correctly destroyed
  // when the loop is destroyed.
  //
  // The websocket thread then spinloops until it sees that mode_ has been
  // set to kFinished. In relevant places on the main thread (i.e., in the
  // ThrowIfWebsocketThreadExited failure poll, or in the constructor when
  // throwing, or during the destructor as normal), it sets mode_ to
  // kFinished to indicate that it will never post into the loop again.

  enum OperatingMode {
    // The main thread and websocket thread are operating as normal.
    kNominal,

    // The websocket thread is no longer running the event loop. It is paused
    // waiting for the main thread to acknowledge the that the event loop is
    // no longer running.
    kStopping,

    // Once this mode is reached, the main thread will not perform any more
    // operations on the websocket thread, other than joining it. Therefore,
    // in this mode the main thread is not allowed to refer to the loop_
    // pointer (it might be nullptr).
    kFinished,
  };
  mutable std::atomic<OperatingMode> mode_{kNominal};

  // These bools are used during unit testing to inject exceptions into various
  // places on the websocket thread.
  std::atomic<bool> inject_open_fault_{false};
  std::atomic<bool> inject_message_fault_{false};
};

namespace {
MeshcatParams MakeMeshcatParamsPortOnly(std::optional<int> port) {
  MeshcatParams result;
  result.port = port;
  return result;
}
}  // namespace

Meshcat::Meshcat(std::optional<int> port)
    : Meshcat(MakeMeshcatParamsPortOnly(port)) {}

Meshcat::Meshcat(const MeshcatParams& params)
    // Creates the server thread, bind to the port, etc.
    : impl_{new Impl(params)} {
  drake::log()->info("Meshcat listening for connections at {}", web_url());
}

Meshcat::~Meshcat() {
  delete static_cast<Impl*>(impl_);
}

// This overloaded function ensures that ThrowIfWebsocketThreadExited always
// happens before accessing the Impl class.
Meshcat::Impl& Meshcat::impl() {
  DRAKE_DEMAND(impl_ != nullptr);
  Impl* result = static_cast<Impl*>(impl_);
  result->ThrowIfWebsocketThreadExited();
  return *result;
}

const Meshcat::Impl& Meshcat::impl() const {
  return const_cast<Meshcat*>(this)->impl();
}

std::string Meshcat::web_url() const {
  return impl().web_url();
}

int Meshcat::port() const {
  return impl().port();
}

std::string Meshcat::ws_url() const {
  return impl().ws_url();
}

int Meshcat::GetNumActiveConnections() const {
  return impl().GetNumActiveConnections();
}

void Meshcat::Flush() const {
  impl().Flush();
}

void Meshcat::SetObject(std::string_view path, const Shape& shape,
                        const Rgba& rgba) {
  impl().SetObject(path, shape, rgba);
}

void Meshcat::SetObject(std::string_view path,
                        const perception::PointCloud& cloud, double point_size,
                        const Rgba& rgba) {
  impl().SetObject(path, cloud, point_size, rgba);
}

void Meshcat::SetObject(std::string_view path,
                        const TriangleSurfaceMesh<double>& mesh,
                        const Rgba& rgba, bool wireframe,
                        double wireframe_line_width) {
  impl().SetObject(path, mesh, rgba, wireframe, wireframe_line_width);
}

void Meshcat::SetLine(std::string_view path,
                      const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                      double line_width, const Rgba& rgba) {
  impl().SetLine(path, vertices, line_width, rgba);
}

void Meshcat::SetLineSegments(std::string_view path,
                              const Eigen::Ref<const Eigen::Matrix3Xd>& start,
                              const Eigen::Ref<const Eigen::Matrix3Xd>& end,
                              double line_width, const Rgba& rgba) {
  impl().SetLineSegments(path, start, end, line_width, rgba);
}

void Meshcat::SetTriangleMesh(
    std::string_view path, const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
    const Eigen::Ref<const Eigen::Matrix3Xi>& faces, const Rgba& rgba,
    bool wireframe, double wireframe_line_width) {
  impl().SetTriangleMesh(path, vertices, faces, rgba, wireframe,
                              wireframe_line_width);
}

void Meshcat::SetTriangleColorMesh(
    std::string_view path, const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
    const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
    const Eigen::Ref<const Eigen::Matrix3Xd>& colors, bool wireframe,
    double wireframe_line_width) {
  impl().SetTriangleColorMesh(path, vertices, faces, colors, wireframe,
                         wireframe_line_width);
}

void Meshcat::SetCamera(PerspectiveCamera camera, std::string path) {
  impl().SetCamera(std::move(camera), std::move(path));
}

void Meshcat::SetCamera(OrthographicCamera camera, std::string path) {
  impl().SetCamera(std::move(camera), std::move(path));
}

void Meshcat::SetTransform(std::string_view path,
                           const RigidTransformd& X_ParentPath) {
  impl().SetTransform(path, X_ParentPath);
}

void Meshcat::SetTransform(std::string_view path,
                           const Eigen::Ref<const Eigen::Matrix4d>& matrix) {
  impl().SetTransform(path, matrix);
}

void Meshcat::Delete(std::string_view path) {
  impl().Delete(path);
}

void Meshcat::SetRealtimeRate(double rate) {
  impl().SetRealtimeRate(rate);
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          bool value) {
  impl().SetProperty(path, std::move(property), value);
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          double value) {
  impl().SetProperty(path, std::move(property), value);
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          const std::vector<double>& value) {
  impl().SetProperty(path, std::move(property), value);
}

void Meshcat::SetAnimation(const MeshcatAnimation& animation) {
  impl().SetAnimation(animation);
}

void Meshcat::Set2dRenderMode(const math::RigidTransformd& X_WC, double xmin,
                              double xmax, double ymin, double ymax) {
  impl().Set2dRenderMode(X_WC, xmin, xmax, ymin, ymax);
}

void Meshcat::ResetRenderMode() {
  impl().ResetRenderMode();
}

void Meshcat::AddButton(std::string name, std::string keycode) {
  impl().AddButton(std::move(name), std::move(keycode));
}

int Meshcat::GetButtonClicks(std::string_view name) {
  return impl().GetButtonClicks(name);
}

void Meshcat::DeleteButton(std::string name) {
  impl().DeleteButton(std::move(name));
}

void Meshcat::AddSlider(std::string name, double min, double max, double step,
                        double value, std::string decrement_keycode,
                        std::string increment_keycode) {
  impl().AddSlider(std::move(name), min, max, step, value,
                   std::move(decrement_keycode), std::move(increment_keycode));
}

void Meshcat::SetSliderValue(std::string name, double value) {
  impl().SetSliderValue(std::move(name), value);
}

double Meshcat::GetSliderValue(std::string_view name) {
  return impl().GetSliderValue(name);
}

void Meshcat::DeleteSlider(std::string name) {
  impl().DeleteSlider(std::move(name));
}

void Meshcat::DeleteAddedControls() {
  impl().DeleteAddedControls();
}

std::string Meshcat::StaticHtml() {
  return impl().StaticHtml();
}

bool Meshcat::HasPath(std::string_view path) const {
  return impl().HasPath(path);
}

std::string Meshcat::GetPackedObject(std::string_view path) const {
  return impl().GetPackedObject(path);
}

std::string Meshcat::GetPackedTransform(std::string_view path) const {
  return impl().GetPackedTransform(path);
}

std::string Meshcat::GetPackedProperty(std::string_view path,
                                       std::string property) const {
  return impl().GetPackedProperty(path, std::move(property));
}

void Meshcat::InjectWebsocketThreadFault(int fault_number) {
  impl().InjectWebsocketThreadFault(fault_number);
}

}  // namespace geometry
}  // namespace drake
