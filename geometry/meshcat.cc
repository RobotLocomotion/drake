#include "drake/geometry/meshcat.h"

#include <algorithm>
#include <cctype>
#include <exception>
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

#include "drake/common/filesystem.h"
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
  static const drake::never_destroyed<std::string> meshcat_js(
      LoadResource("drake/geometry/meshcat.js"));
  static const drake::never_destroyed<std::string> meshcat_ico(
      LoadResource("drake/geometry/meshcat.ico"));
  static const drake::never_destroyed<std::string> meshcat_html(
      LoadResource("drake/geometry/meshcat.html"));
  if (url_path == "/meshcat.js") {
    return meshcat_js.access();
  }
  if (url_path == "/favicon.ico") {
    return meshcat_ico.access();
  }
  return meshcat_html.access();
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

  explicit MeshcatShapeReifier(uuids::uuid_random_generator* uuid_generator)
      : uuid_generator_(uuid_generator) {
    DRAKE_DEMAND(uuid_generator != nullptr);
  }

  ~MeshcatShapeReifier() = default;

  using ShapeReifier::ImplementGeometry;

  void ImplementGeometry(const Sphere& sphere, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);
    lumped.object = internal::MeshData();

    auto geometry = std::make_unique<internal::SphereGeometryData>();
    geometry->uuid = uuids::to_string((*uuid_generator_)());
    geometry->radius = sphere.radius();
    lumped.geometry = std::move(geometry);
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

  void ImplementGeometry(const HalfSpace&, void*) override {
    // TODO(russt): Use PlaneGeometry with fields width, height,
    // widthSegments, heightSegments
    drake::log()->warn("Meshcat does not display HalfSpace geometry (yet).");
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

  void ImplementGeometry(const Capsule&, void*) override {
    drake::log()->warn("Meshcat does not display Capsule geometry (yet).");
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

  template <typename T>
  void ImplementMesh(const T& mesh, void* data) {
    DRAKE_DEMAND(data != nullptr);
    auto& lumped = *static_cast<internal::LumpedObjectData*>(data);

    // TODO(russt): Use file contents to generate the uuid, and avoid resending
    // meshes unless necessary.  Using the filename is tempting, but that leads
    // to problems when the file contents change on disk.

    const filesystem::path filename(mesh.filename());
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
      const filesystem::path basedir = filename.parent_path();

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
        // TODO(russt): Make this parsing more robust.
        std::regex map_regex("map_[^\\s]+\\s+([^\\s]+)");
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

  void ImplementGeometry(const Mesh& mesh, void* data) override {
    ImplementMesh(mesh, data);
  }

  void ImplementGeometry(const Convex& mesh, void* data) override {
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

 private:
  uuids::uuid_random_generator* const uuid_generator_{};
};

}  // namespace

class Meshcat::WebSocketPublisher {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(WebSocketPublisher);

  explicit WebSocketPublisher(const std::optional<int> port)
      : prefix_("/drake"), main_thread_id_(std::this_thread::get_id()) {
    DRAKE_DEMAND(!port.has_value() || *port >= 1024);
    std::promise<std::tuple<uWS::Loop*, int, bool>> app_promise;
    std::future<std::tuple<uWS::Loop*, int, bool>> app_future =
        app_promise.get_future();
    websocket_thread_ = std::thread(&WebSocketPublisher::WebSocketMain, this,
                                    std::move(app_promise), port);
    bool connected;
    std::tie(loop_, port_, connected) = app_future.get();

    if (!connected) {
      websocket_thread_.join();
      throw std::runtime_error("Meshcat failed to open a websocket port.");
    }
  }

  ~WebSocketPublisher() {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);
    loop_->defer([this]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      auto iter = websockets_.begin();
      while (iter != websockets_.end()) {
        // Need to advance the iterator before calling close (#15821).
        auto* ws = *iter++;
        ws->close();
      }
      us_listen_socket_close(0, listen_socket_);
    });
    websocket_thread_.join();
  }

  int port() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    return port_;
  }

  void SetObject(std::string_view path, const Shape& shape, const Rgba& rgba) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

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
      material->color = (static_cast<int>(255 * rgba.r()) << 16) +
                      (static_cast<int>(255 * rgba.g()) << 8) +
                      static_cast<int>(255 * rgba.b());
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

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      // TODO(russt): Consider using msgpack::sbuffer instead of stringstream
      // (here and throughout) to avoid this copy.
      // https://github.com/redboltz/msgpack-c/wiki/v2_0_cpp_packer
      std::string message = message_stream.str();
      if (message.size() > kMaxBackPressure) {
        drake::log()->warn(
            "The message describing the object at {} is too large for the "
            "current websocket setup (size {} is greater than the max "
            "backpressure {}).  You will either need to reduce the size of "
            "your object/mesh/textures, or modify the code to increase the "
            "allowance.",
            data.path, message.size(), kMaxBackPressure);
      }
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  void SetObject(std::string_view path, const perception::PointCloud& cloud,
                 double point_size, const Rgba& rgba) {
    DRAKE_DEMAND(std::this_thread::get_id() == main_thread_id_);
    DRAKE_DEMAND(loop_ != nullptr);

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
    material->color = (static_cast<int>(255 * rgba.r()) << 16) +
                      (static_cast<int>(255 * rgba.g()) << 8) +
                      static_cast<int>(255 * rgba.b());
    material->size = point_size;
    material->vertexColors = cloud.has_rgbs();
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuids::to_string(uuid_generator());
    mesh.type = "Points";
    mesh.geometry = data.object.geometry->uuid;
    mesh.material = data.object.material->uuid;
    data.object.object = std::move(mesh);

    loop_->defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      if (message.size() > kMaxBackPressure) {
        drake::log()->warn(
            "The message describing the object at {} is too large for the "
            "current websocket setup (size {} is greater than the max "
            "backpressure {}).  You will either need to reduce the size of "
            "your object/mesh/textures, or modify the code to increase the "
            "allowance.",
            data.path, message.size(), kMaxBackPressure);
      }
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object() = std::move(message);
    });
  }

  template <typename CameraData>
  void SetCamera(CameraData camera, std::string path) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    uuids::uuid_random_generator uuid_generator{generator_};
    internal::SetCameraData<CameraData> data;
    data.path = std::move(path);
    data.object.object = std::move(camera);

    loop_->defer([this, data = std::move(data)]() {
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

  void SetTransform(std::string_view path,
                    const RigidTransformd& X_ParentPath) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::SetTransformData data;
    data.path = FullPath(path);
    Eigen::Map<Eigen::Matrix4d>(data.matrix) = X_ParentPath.GetAsMatrix4();

    loop_->defer([this, data = std::move(data)]() {
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

  void Delete(std::string_view path) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::DeleteData data;
    data.path = FullPath(path);

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
      scene_tree_root_.Delete(data.path);
    });
  }

  template <typename T>
  void SetProperty(std::string_view path, std::string property,
                   const T& value) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::SetPropertyData<T> data;
    data.path = FullPath(path);
    data.property = std::move(property);
    data.value = value;

    loop_->defer([this, data = std::move(data)]() {
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

  void SetAnimation(const MeshcatAnimation& animation) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    std::stringstream message_stream;
    // We pack this message in-place (rather than using structs to organize the
    // packing) for a few reasons:
    //  1) we want to avoid copying the big data nested structure,
    //  2) this message type would require a nasty hairball of structs, and
    //  3) the nested structures have path's inside that must be modified with
    //     FullPath().
    msgpack::packer o(message_stream);
    // The details of this message have been extracted primarily from
    // meshcat/test/animation.html and meshcat-python/src/meshcat/animation.py.
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

    loop_->defer(
        [this, message = message_stream.str()]() {
          DRAKE_DEMAND(IsThread(websocket_thread_id_));
          DRAKE_DEMAND(app_ != nullptr);
          app_->publish("all", message, uWS::OpCode::BINARY, false);
          animation_ = std::move(message);
        });
  }

  void AddButton(std::string name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::SetButtonControl data;
    data.callback = fmt::format(R"""(
() => this.connection.send(msgpack.encode({{
  'type': 'button',
  'name': '{}'
}})))""",
                                name);
    data.name = std::move(name);

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      if (buttons_.find(name) != buttons_.end()) {
        DeleteButton(name);
      }
      if (sliders_.find(name) != sliders_.end()) {
        DeleteSlider(name);
      }
      controls_.emplace_back(data.name);
      buttons_[data.name] = data;
    }

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  int GetButtonClicks(std::string_view name) {
    std::lock_guard<std::mutex> lock(controls_mutex_);
    auto iter = buttons_.find(name);
    if (iter == buttons_.end()) {
      throw std::out_of_range(
          fmt::format("Meshcat does not have any button named {}.", name));
    }
    return iter->second.num_clicks;
  }

  void DeleteButton(std::string name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::DeleteControl data;
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = buttons_.find(name);
      if (iter == buttons_.end()) {
        throw std::out_of_range(
            fmt::format("Meshcat does not have any button named {}.", name));
      }
      buttons_.erase(iter);
      auto c_iter = std::find(controls_.begin(), controls_.end(), name);
      DRAKE_DEMAND(c_iter != controls_.end());
      controls_.erase(c_iter);
      data.name = std::move(name);
    }

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  void AddSlider(std::string name, double min, double max,
                               double step, double value) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::SetSliderControl data;
    data.callback = fmt::format(R"""(
(value) => this.connection.send(msgpack.encode({{
  'type': 'slider',
  'name': '{}',
  'value': value
}})))""",
                                name);
    data.name = std::move(name);
    data.min = min;
    data.max = max;
    data.step = step;
    // Match setValue in NumberController.js from dat.GUI.
    // https://github.com/dataarts/dat.gui/blob/f720c729deca5d5c79da8464f8a05500d38b140c/src/dat/controllers/NumberController.js#L62
    value = std::max(value, min);
    value = std::min(value, max);
    value = std::round(value/step)*step;
    data.value = value;

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      if (buttons_.find(name) != buttons_.end()) {
        DeleteButton(name);
      }
      if (sliders_.find(name) != sliders_.end()) {
        DeleteSlider(name);
      }
      controls_.emplace_back(data.name);
      sliders_[data.name] = data;
    }

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  void SetSliderValue(std::string name, double value) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = sliders_.find(name);
      if (iter == sliders_.end()) {
        throw std::out_of_range(
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

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

  double GetSliderValue(std::string_view name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::lock_guard<std::mutex> lock(controls_mutex_);
    auto iter = sliders_.find(name);
    if (iter == sliders_.end()) {
      throw std::out_of_range(
          fmt::format("Meshcat does not have any slider named {}.", name));
    }
    return iter->second.value;
  }

  void DeleteSlider(std::string name) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    internal::DeleteControl data;
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = sliders_.find(name);
      if (iter == sliders_.end()) {
        throw std::out_of_range(
            fmt::format("Meshcat does not have any slider named {}.", name));
      }
      sliders_.erase(iter);
      auto c_iter = std::find(controls_.begin(), controls_.end(), name);
      DRAKE_DEMAND(c_iter != controls_.end());
      controls_.erase(c_iter);
      data.name = std::move(name);
    }

    loop_->defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      app_->publish("all", message_stream.str(), uWS::OpCode::BINARY, false);
    });
  }

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

  bool HasPath(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<bool> p;
    std::future<bool> f = p.get_future();
    loop_->defer([this, path = FullPath(path), p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      p.set_value(scene_tree_root_.Find(path) != nullptr);
    });
    return f.get();
  }

  std::string GetPackedObject(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    loop_->defer([this, path = FullPath(path), p = std::move(p)]() mutable {
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

  std::string GetPackedTransform(std::string_view path) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    loop_->defer([this, path = FullPath(path), p = std::move(p)]() mutable {
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

  std::string GetPackedProperty(std::string_view path,
                                std::string property) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    DRAKE_DEMAND(loop_ != nullptr);

    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    loop_->defer([this, path = FullPath(path), property = std::move(property),
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

 private:
  bool IsThread(std::thread::id thread_id) const {
    return (std::this_thread::get_id() == thread_id);
  }

  void WebSocketMain(
      std::promise<std::tuple<uWS::Loop*, int, bool>> app_promise,
      const std::optional<int>& desired_port) {
    websocket_thread_id_ = std::this_thread::get_id();

    int port = desired_port ? *desired_port : 7000;
    const int kMaxPort = desired_port ? *desired_port : 7099;

    uWS::App::WebSocketBehavior<PerSocketData> behavior;
    behavior.open = [this](WebSocket* ws) {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      websockets_.emplace(ws);
      ws->subscribe("all");
      // Update this new connection with previously published data.
      SendTree(ws);
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
    };
    // TODO(russt): I could increase this more if necessary (when it was too
    // low, some SetObject messages were dropped).  But at some point the real
    // fix is to actually throttle the sending (e.g. by slowing down the main
    // thread).
    behavior.maxBackpressure = kMaxBackPressure;
    behavior.message = [this](WebSocket* ws, std::string_view message,
                              uWS::OpCode op_code) {
      unused(ws, op_code);
      internal::UserInterfaceEvent data;
      try {
        msgpack::object_handle o_h =
            msgpack::unpack(message.data(), message.size());
        o_h.get().convert(data);
      } catch (const std::bad_alloc& e) {
        // Quietly ignore messages that don't match our expected message type.
        // This violates the style guide, but msgpack does not provide any other
        // mechanism for checking the message type.
        return;
      }
      std::lock_guard<std::mutex> lock(controls_mutex_);
      if (data.type == "button") {
        auto iter = buttons_.find(data.name);
        if (iter != buttons_.end()) {
          iter->second.num_clicks++;
        }
      } else if (data.type == "slider" && data.value.has_value()) {
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
      }
    };
    behavior.close = [this](WebSocket* ws, int, std::string_view) {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      websockets_.erase(ws);
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
          port, LIBUS_LISTEN_EXCLUSIVE_PORT,
          [this, port](us_listen_socket_t* socket) {
            DRAKE_DEMAND(IsThread(websocket_thread_id_));
            if (socket) {
              drake::log()->info(
                  "Meshcat listening for connections at http://localhost:{}",
                  port);
              listen_socket_ = socket;
            }
          });
    } while (listen_socket_ == nullptr && port++ < kMaxPort);

    bool connected = listen_socket_ != nullptr;
    app_promise.set_value(std::make_tuple(uWS::Loop::get(), port, connected));

    if (connected) {
      app.run();
    }
  }

  void SendTree(WebSocket* ws) {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));
    scene_tree_root_.Send(ws);
  }

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
  int port_{};
  std::mt19937 generator_{};

  // These variables should only be accessed in the websocket thread.
  std::thread::id websocket_thread_id_{};
  SceneTreeElement scene_tree_root_{};
  std::string animation_;
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

Meshcat::Meshcat(const std::optional<int>& port) {
  // Fetch the index once to be sure that we preload the content.
  GetUrlContent("/");

  publisher_ = std::make_unique<WebSocketPublisher>(port);
}

Meshcat::~Meshcat() = default;

std::string Meshcat::web_url() const {
  return fmt::format("http://localhost:{}", publisher_->port());
}

int Meshcat::port() const {
  return publisher_->port();
}

std::string Meshcat::ws_url() const {
  return fmt::format("ws://localhost:{}", publisher_->port());
}

void Meshcat::SetObject(std::string_view path, const Shape& shape,
                        const Rgba& rgba) {
  publisher_->SetObject(path, shape, rgba);
}

void Meshcat::SetObject(std::string_view path,
                        const perception::PointCloud& cloud, double point_size,
                        const Rgba& rgba) {
  publisher_->SetObject(path, cloud, point_size, rgba);
}

void Meshcat::SetCamera(PerspectiveCamera camera, std::string path) {
  publisher_->SetCamera(std::move(camera), std::move(path));
}

void Meshcat::SetCamera(OrthographicCamera camera, std::string path) {
  publisher_->SetCamera(std::move(camera), std::move(path));
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

void Meshcat::SetProperty(std::string_view path, std::string property,
                          const std::vector<double>& value) {
  publisher_->SetProperty(path, std::move(property), value);
}

void Meshcat::SetAnimation(const MeshcatAnimation& animation) {
  publisher_->SetAnimation(animation);
}

void Meshcat::Set2dRenderMode(const math::RigidTransformd& X_WC, double xmin,
                              double xmax, double ymin, double ymax) {
  // Set orthographic camera.
  OrthographicCamera camera;
  camera.left = xmin;
  camera.right = xmax;
  camera.bottom = ymin;
  camera.top = ymax;
  SetCamera(camera);

  SetTransform("/Cameras/default", X_WC);
  // Lock orbit controls.
  SetProperty("/Cameras/default/rotated/<object>", "position",
              {0.0, 0.0, 0.0});

  SetProperty("/Background", "visible", false);
  SetProperty("/Grid", "visible", false);
  SetProperty("/Axes", "visible", false);
}

void Meshcat::ResetRenderMode() {
  PerspectiveCamera camera;
  SetCamera(camera);
  SetTransform("/Cameras/default", math::RigidTransformd());
  // Lock orbit controls.
  SetProperty("/Cameras/default/rotated/<object>", "position",
              {0.0, 1.0, 3.0});
  SetProperty("/Background", "visible", true);
  SetProperty("/Grid", "visible", true);
  SetProperty("/Axes", "visible", true);
}

void Meshcat::AddButton(std::string name) {
  publisher_->AddButton(std::move(name));
}

int Meshcat::GetButtonClicks(std::string_view name) {
  return publisher_->GetButtonClicks(name);
}

void Meshcat::DeleteButton(std::string name) {
  publisher_->DeleteButton(std::move(name));
}

void Meshcat::AddSlider(std::string name, double min, double max,
                               double step, double value) {
  publisher_->AddSlider(std::move(name), min, max, step, value);
}

void Meshcat::SetSliderValue(std::string name, double value) {
  publisher_->SetSliderValue(std::move(name), value);
}

double Meshcat::GetSliderValue(std::string_view name) {
  return publisher_->GetSliderValue(name);
}

void Meshcat::DeleteSlider(std::string name) {
  publisher_->DeleteSlider(std::move(name));
}

void Meshcat::DeleteAddedControls() {
  publisher_->DeleteAddedControls();
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
