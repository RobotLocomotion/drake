#include "drake/geometry/meshcat.h"

#include <algorithm>
#include <atomic>
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
#include <fmt/ranges.h>
#include <msgpack.hpp>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"
#include "drake/common/find_resource.h"
#include "drake/common/network_policy.h"
#include "drake/common/overloaded.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"
#include "drake/geometry/meshcat_file_storage_internal.h"
#include "drake/geometry/meshcat_internal.h"
#include "drake/geometry/meshcat_recording_internal.h"
#include "drake/geometry/meshcat_types_internal.h"
#include "drake/geometry/meshcat_zip_factory_internal.h"
#include "drake/geometry/proximity/polygon_to_triangle_mesh.h"
#include "drake/systems/analysis/realtime_rate_calculator.h"

#ifdef BOOST_VERSION
#error Drake should be using the non-boost flavor of msgpack.
#endif

// Steal one function declaration from usockets/src/internal/internal.h.
extern "C" {
void us_internal_free_closed_sockets(struct us_loop_t*);
}

namespace fs = std::filesystem;

namespace drake {
namespace geometry {
namespace {

using internal::FileStorage;
using internal::MeshcatZipFactory;
using math::RigidTransformd;
using math::RotationMatrixd;

template <typename Mapping>
[[noreturn]] void ThrowThingNotFound(std::string_view thing,
                                     std::string_view name,
                                     const Mapping& thing_map) {
  std::vector<std::string> keys;
  for (const auto& map_pair : thing_map) {
    keys.push_back(map_pair.first);
  }
  throw std::logic_error(
      fmt::format("Meshcat does not have any {} named {}."
                  " The registered {} names are ({}).",
                  thing, name, thing, fmt::join(keys, ", ")));
}

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
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SceneTreeElement);

  SceneTreeElement() = default;

  /* Each SceneTreeElement stores the set of messages that would re-create it
  from scratch. Some of those messages might refer to asset files. This struct
  stores the message bytes alongside any asset handles that the message refers
  to, so it's easier to keep the two pieces in sync. */
  struct Message {
    DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(Message);

    Message() = default;

    /* Convenience operator to assign to `bytes` (and therefore, clear any
    `assets` used by the prior value of bytes). The assets are unconditionally
    cleared -- even if `new_bytes` is the same as the prior `bytes. */
    Message& operator=(std::string&& new_bytes) {
      bytes = std::move(new_bytes);
      assets.clear();
      return *this;
    }

    /* The packed command message that conveys some portion of this
    SceneTreeElement to meshcat.js. */
    std::string bytes;

    /* If the message refers to http assets (e.g., image files), then this list
    is responsible for keeping alive a non-zero reference count for those
    file(s) in our in-memory storage. */
    std::vector<std::shared_ptr<const MemoryFile>> assets;
  };

  // Provide direct access to all member fields except the list of children.
  const std::optional<Message>& object() const { return object_; }
  std::optional<Message>& object() { return object_; }
  const std::optional<Message>& transform() const { return transform_; }
  std::optional<Message>& transform() { return transform_; }
  const std::map<std::string, Message>& properties() const { return props_; }
  std::map<std::string, Message>& properties() { return props_; }

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
      ws->send(object_->bytes);
    }
    if (transform_) {
      ws->send(transform_->bytes);
    }
    for (const auto& [_, property] : props_) {
      ws->send(property.bytes);
    }

    for (const auto& [_, child] : children_) {
      child->Send(ws);
    }
  }

  // Returns a string which implements the entire tree directly in javascript.
  // This is intended for use in generating a "static html" of the scene.
  std::string CreateCommands() const {
    // N.B. The string::operator+= here might look like a performance problem,
    // but string appending actually uses exponential growth (i.e., similar to
    // how std::vector<char> works) so the overhead here is not bad.
    std::string html;
    if (object_) {
      html += CreateCommand(object_->bytes);
    }
    if (transform_) {
      html += CreateCommand(transform_->bytes);
    }
    for (const auto& [_, property] : props_) {
      html += CreateCommand(property.bytes);
    }
    for (const auto& [_, child] : children_) {
      html += child->CreateCommands();
    }
    return html;
  }

 private:
  // Note: We use std::optional here to clearly denote the variables that have
  // not been set, and therefore need not be sent over the websocket.

  // The msgpack'd set_object command.
  std::optional<Message> object_;
  // The msgpack'd set_transform command.
  std::optional<Message> transform_;
  // The msgpack'd set_property command(s).
  std::map<std::string, Message> props_;
  // Children, with the key value denoting their (relative) path name.
  std::map<std::string, std::unique_ptr<SceneTreeElement>> children_;
};

int ToMeshcatColor(const Rgba& rgba) {
  // Note: The returned color discards the alpha value, which is handled
  // separately (e.g. by the opacity field in the material properties).
  return (static_cast<int>(255 * rgba.r()) << 16) +
         (static_cast<int>(255 * rgba.g()) << 8) +
         static_cast<int>(255 * rgba.b());
}

// Sets the lumped object's geometry, material, and object type based on the
// mesh data and its material properties.
void SetLumpedObjectFromTriangleMesh(
    internal::LumpedObjectData* object,
    const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
    const Eigen::Ref<const Eigen::Matrix3Xi>& faces, const Rgba& rgba,
    bool wireframe, double wireframe_line_width,
    Meshcat::SideOfFaceToRender side, internal::UuidGenerator* uuid_generator) {
  DRAKE_DEMAND(object != nullptr);
  DRAKE_DEMAND(uuid_generator != nullptr);

  auto geometry = std::make_unique<internal::BufferGeometryData>();
  geometry->uuid = uuid_generator->GenerateRandom();
  geometry->position = vertices.cast<float>();
  geometry->faces = faces.cast<uint32_t>();
  object->geometry = std::move(geometry);

  auto material = std::make_unique<internal::MaterialData>();
  material->uuid = uuid_generator->GenerateRandom();
  material->type = "MeshPhongMaterial";
  material->color = ToMeshcatColor(rgba);
  material->transparent = (rgba.a() != 1.0);
  material->opacity = rgba.a();
  material->wireframe = wireframe;
  material->wireframeLineWidth = wireframe_line_width;
  material->vertexColors = false;
  material->side = side;
  material->flatShading = true;
  object->material = std::move(material);

  internal::MeshData mesh;
  mesh.uuid = uuid_generator->GenerateRandom();
  mesh.type = "Mesh";
  mesh.geometry = object->geometry->uuid;
  mesh.material = object->material->uuid;
  object->object = std::move(mesh);
}

class MeshcatShapeReifier : public ShapeReifier {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshcatShapeReifier);

  MeshcatShapeReifier(internal::UuidGenerator* uuid_generator,
                      FileStorage* file_storage, Rgba rgba)
      : uuid_generator_(*uuid_generator),
        file_storage_(*file_storage),
        rgba_(rgba) {
    DRAKE_DEMAND(uuid_generator != nullptr);
    DRAKE_DEMAND(file_storage != nullptr);
  }

  ~MeshcatShapeReifier() = default;

  // This encapsulates the "return value" of the reifier. The `void*` argument
  // to ImplementGeometry() should always be passed as this type.
  struct Output {
    internal::LumpedObjectData& lumped;
    std::vector<std::shared_ptr<const MemoryFile>>& assets;
  };

  using ShapeReifier::ImplementGeometry;

  // TODO(SeanCurtis-TRI): In follow up commit, move this down in alphabetical
  // order.
  void ImplementGeometry(const Mesh& mesh, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    // TODO(russt): Use file contents to generate the uuid, and avoid resending
    // meshes unless necessary.  Using the filename is tempting, but that leads
    // to problems when the file contents change on disk.

    std::string format = mesh.extension();
    format.erase(0, 1);  // remove the . from the extension

    const MeshSource& mesh_source = mesh.source();

    // Precompute the basedir which we'll only use if source.is_path().
    const fs::path basedir_if_path =
        mesh_source.is_path() ? mesh_source.path().parent_path() : fs::path();

    // We simply dump the binary contents of the file into the data field of the
    // message. The javascript meshcat takes care of the rest, but first we
    // need to acquire a copy -- either from a file or from an in-memory string.
    std::string mesh_data;
    if (mesh_source.is_path()) {
      std::optional<std::string> maybe_mesh_data = ReadFile(mesh_source.path());
      if (!maybe_mesh_data) {
        drake::log()->warn("Meshcat: Could not open mesh filename {}",
                           mesh_source.path().string());
        return;
      }
      mesh_data = std::move(*maybe_mesh_data);
    } else {
      DRAKE_DEMAND(mesh_source.is_in_memory());
      mesh_data = mesh_source.in_memory().mesh_file.contents();
    }

    // TODO(russt): MeshCat.jl/src/mesh_files.jl loads dae with textures, also.

    // TODO(russt): Make this mtllib parsing more robust (right now commented
    // mtllib lines will match, too, etc).
    size_t mtllib_pos;
    bool use_meshfile_geometry = false;
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
          lumped.object.emplace<internal::MeshfileObjectData>();
      meshfile_object.uuid = uuid_generator_.GenerateRandom();
      meshfile_object.format = std::move(format);
      meshfile_object.data = std::move(mesh_data);

      const std::string mtllib = matches.str(1);

      std::optional<std::string> maybe_mtl_data;
      if (mesh_source.is_path()) {
        DRAKE_DEMAND(!basedir_if_path.empty());
        maybe_mtl_data = ReadFile(basedir_if_path / mtllib);
      } else {
        const auto mtl_iter =
            mesh_source.in_memory().supporting_files.find(mtllib);
        if (mtl_iter != mesh_source.in_memory().supporting_files.end()) {
          maybe_mtl_data = std::visit<std::optional<std::string>>(
              overloaded{[](const fs::path& path) {
                           return ReadFile(path);
                         },
                         [](const MemoryFile& file) {
                           return file.contents();
                         }},
              mtl_iter->second);
        }
      }

      // Read .mtl file into geometry.mtl_library.
      if (maybe_mtl_data.has_value()) {
        meshfile_object.mtl_library = std::move(*maybe_mtl_data);

        // Scan .mtl file for map_ lines.  For each, load the file and add
        // the contents to geometry.resources.
        // The syntax (http://paulbourke.net/dataformats/mtl/) is e.g.
        //   map_Ka -options args image_name
        // Here we ignore the options and only extract the image_name (by
        // extracting the last word before the end of line/string).
        //  - "map_.+" matches the map_ plus any options,
        //  - "\s" matches one whitespace (before the image_name),
        //  - "[^\s]+" matches the image_name, and
        //  - "(?:$|\r|\n)" matches the end of string or end of line.
        // TODO(russt): This parsing could still be more robust.
        std::regex map_regex(R"""(map_.+\s([^\s]+)\s*(?:$|\r|\n))""");
        for (std::sregex_iterator iter(meshfile_object.mtl_library.begin(),
                                       meshfile_object.mtl_library.end(),
                                       map_regex);
             iter != std::sregex_iterator(); ++iter) {
          const std::string map = iter->str(1);
          // The *possible* path to the texture image; non-empty if we need to
          // read the image from disk.
          fs::path maybe_map_path;
          // We'll put the bytes of the available image in here.
          std::vector<uint8_t> map_data;
          if (mesh_source.is_in_memory()) {
            const auto map_file_iter =
                mesh_source.in_memory().supporting_files.find(map);
            if (map_file_iter !=
                mesh_source.in_memory().supporting_files.end()) {
              // Load it.
              std::visit(overloaded{[&maybe_map_path](const fs::path& path) {
                                      // Note: paths to supporting files in an
                                      // in-memory mesh have no "base
                                      // directory". The path must be
                                      // sufficiently well defined so that it
                                      // can be read directly.
                                      maybe_map_path = path;
                                    },
                                    [&map_data](const MemoryFile& file) {
                                      map_data = std::vector<uint8_t>(
                                          file.contents().begin(),
                                          file.contents().end());
                                    }},
                         map_file_iter->second);
            }
          } else {
            DRAKE_DEMAND(mesh_source.is_path());
            maybe_map_path = basedir_if_path / map;
          }

          // maybe_map_path is non-empty only if one of the paths above
          // indicated the image is on disk.
          if (!maybe_map_path.empty()) {
            std::ifstream map_stream(maybe_map_path,
                                     std::ios::binary | std::ios::ate);
            if (map_stream.is_open()) {
              int map_size = map_stream.tellg();
              map_stream.seekg(0, std::ios::beg);
              map_data.reserve(map_size);
              map_data.assign(std::istreambuf_iterator<char>(map_stream),
                              std::istreambuf_iterator<char>());
            }
          }

          // Either we now have bytes for the map, or we had a look-up error.
          if (map_data.size() > 0) {
            meshfile_object.resources.try_emplace(
                map, std::string("data:image/png;base64,") +
                         common_robotics_utilities::base64_helpers::Encode(
                             map_data));
          } else {
            drake::log()->warn(
                "Meshcat: Failed to load texture. \"{}\" references '{}', but "
                "Meshcat could not open filename \"{}\"",
                (basedir_if_path / mtllib).string(), map,
                maybe_map_path.string());
          }
        }
      } else if (!mtllib.empty()) {
        drake::log()->warn(
            "Meshcat: An obj referenced a material library '{}' that Meshcat "
            "could not open; no materials will be included. Obj: '{}'.",
            mtllib, mesh_source.description());

        // If we can't load the mtl file, we'll just send the obj file as if it
        // did not specify the mtl. MuJoCo often ships obj files that reference
        // missing mtl files (see #20444).
        use_meshfile_geometry = true;
        // Move the data back.
        format = std::move(meshfile_object.format);
        mesh_data = std::move(meshfile_object.data);
      }
    } else if (format == "gltf") {
      output.assets =
          internal::UnbundleGltfAssets(mesh_source, &mesh_data, &file_storage_);
      auto& meshfile_object =
          lumped.object.emplace<internal::MeshfileObjectData>();
      meshfile_object.uuid = uuid_generator_.GenerateRandom();
      meshfile_object.format = std::move(format);
      meshfile_object.data = std::move(mesh_data);
    } else {
      // We have a mesh that isn't a .gltf nor an obj with mtl. So, we'll make
      // mesh file *geometry* instead of mesh file *object*. This will most
      // typically be a Collada .dae file, an .stl, or simply an .obj that
      // doesn't reference an .mtl.
      use_meshfile_geometry = true;
    }

    if (use_meshfile_geometry) {
      // TODO(SeanCurtis-TRI): This doesn't work for STL even though meshcat
      // supports STL. Meshcat treats STL differently from obj or dae.
      // https://github.com/meshcat-dev/meshcat/blob/4b4f8ffbaa5f609352ea6227bd5ae8207b579c70/src/index.js#L130-L146.
      // The "data" property of the _meshfile_geometry for obj and dae are
      // simply passed along verbatim. But for STL it is interpreted as a
      // buffer. However, we're not passing the data in a way that deserializes
      // into a data array. So, either meshcat needs to change how it gets
      // STL (being more permissive), or we need to change how we transmit STL
      // data.

      // TODO(SeanCurtis-TRI): Provide test showing that .dae works.

      if (format != "obj" && format != "dae") {
        // Note: We send the data along to meshcat regardless relying on meshcat
        // to ignore the mesh and move on. The *path* will still exist.
        static const logging::Warn one_time(
            "Drake's Meshcat only supports Mesh/Convex specifications which "
            "use .obj, .gltf, or .dae files. Mesh specifications using other "
            "mesh types (e.g., .stl, etc.) will not be visualized.");
      }
      auto geometry = std::make_unique<internal::MeshFileGeometryData>();
      geometry->uuid = uuid_generator_.GenerateRandom();
      geometry->format = std::move(format);
      geometry->data = std::move(mesh_data);
      lumped.geometry = std::move(geometry);

      lumped.object.emplace<internal::MeshData>();
    }

    // Set the scale. Note that if this were a general transform including a
    // rotation we would have to _multiply_ the diagonal by the scale factors.
    // In meshcat, rotation is applied to a different node, so we can safely
    // treat the lumped_object's matrix as if it contained an identity rotation
    // and safely directly _set_ the scale factors on the diagonal.
    const Vector3<double>& scale = mesh.scale3();
    std::visit<void>(
        overloaded{[](std::monostate) {},
                   [scale](auto& lumped_object) {
                     Eigen::Map<Eigen::Matrix4d> matrix(lumped_object.matrix);
                     matrix(0, 0) = scale.x();
                     matrix(1, 1) = scale.y();
                     matrix(2, 2) = scale.z();
                   }},
        lumped.object);
  }

  void ImplementGeometry(const Box& box, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    lumped.object = internal::MeshData();

    auto geometry = std::make_unique<internal::BoxGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->width = box.width();
    // Three.js uses height for the y axis; Drake uses depth.
    geometry->height = box.depth();
    geometry->depth = box.height();
    lumped.geometry = std::move(geometry);
  }

  void ImplementGeometry(const Capsule& capsule, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::CapsuleGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radius = capsule.radius();
    geometry->height = capsule.length();
    lumped.geometry = std::move(geometry);

    // Meshcat cylinders have their long axis in y.
    Eigen::Map<Eigen::Matrix4d>(mesh.matrix) =
        RigidTransformd(RotationMatrixd::MakeXRotation(M_PI / 2.0))
            .GetAsMatrix4();
  }

  void ImplementGeometry(const Convex& mesh, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& output = *static_cast<Output*>(data);

    const PolygonSurfaceMesh<double>& hull = mesh.GetConvexHull();
    const TriangleSurfaceMesh<double> tri_hull =
        internal::MakeTriangleFromPolygonMesh(hull);

    Eigen::Matrix3Xd vertices(3, tri_hull.num_vertices());
    for (int i = 0; i < tri_hull.num_vertices(); ++i) {
      vertices.col(i) = tri_hull.vertex(i);
    }
    Eigen::Matrix3Xi faces(3, tri_hull.num_triangles());
    for (int i = 0; i < tri_hull.num_triangles(); ++i) {
      const auto& e = tri_hull.element(i);
      for (int j = 0; j < 3; ++j) {
        faces(j, i) = e.vertex(j);
      }
    }
    SetLumpedObjectFromTriangleMesh(&output.lumped, vertices, faces, rgba_,
                                    /* wireframe =*/false, 1.0,
                                    Meshcat::SideOfFaceToRender::kDoubleSide,
                                    &uuid_generator_);
  }

  void ImplementGeometry(const Cylinder& cylinder, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::CylinderGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
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
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::SphereGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
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

  void ImplementGeometry(const MeshcatCone& cone, void* data) override {
    DRAKE_DEMAND(data != nullptr);
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    auto& mesh = lumped.object.emplace<internal::MeshData>();

    auto geometry = std::make_unique<internal::CylinderGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
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
    auto& output = *static_cast<Output*>(data);
    auto& lumped = output.lumped;
    lumped.object = internal::MeshData();

    auto geometry = std::make_unique<internal::SphereGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->radius = sphere.radius();
    lumped.geometry = std::move(geometry);
  }

 private:
  internal::UuidGenerator& uuid_generator_;
  FileStorage& file_storage_;
  Rgba rgba_;
};

// Meshcat inherits three.js's y-up world and it is applied to camera and
// camera target positions. To simply set the object's position property, we
// need to express the position in three.js's y-up world frame.
// It's simply a 90-degree rotation around the x-axis, so we hard-code it here.
Eigen::Vector3d MeshcatYUpPosition(const Eigen::Vector3d& p_WP) {
  return Eigen::Vector3d(p_WP.x(), p_WP.z(), -p_WP.y());
}

// Creates a Drake camera pose from a meshcat camera pose (encoded as an array
// of 16 values).
// @pre the 16 values form a valid rigid transform (i.e., the rotation matrix
// is properly orthonormal -- the scale is the Identity, etc.).
RigidTransformd MakeDrakePoseFromMeshcatPoseForCamera(
    const std::vector<double>& values) {
  DRAKE_DEMAND(values.size() == 16);
  Eigen::Map<const Eigen::Matrix4d> matrix(values.data());
  // The pose of the meshcat camera in meshcat's y-up world frame M. We're
  // treating the bottom row of the matrix as [0, 0, 0, 1] but otherwise
  // ignoring it.
  const RigidTransformd X_MCm(matrix.block<3, 4>(0, 0));
  DRAKE_DEMAND(X_MCm.rotation().IsValid());

  // Meshcat (aka three.js) cameras are y-up. Rotate to be z-up for Drake.
  const auto R_WM = RotationMatrixd::MakeXRotation(M_PI / 2);
  const Eigen::Vector3d p_WC = R_WM * X_MCm.translation();

  // Meshcat camera aims in the -Cz direction with Cy up. Drake renders in the
  // +Cy direction with -Cy up. So, we need to rotate again.
  const auto R_CdCm = RotationMatrixd(math::RollPitchYawd(0, M_PI, M_PI));
  const auto R_WCd = R_WM * X_MCm.rotation() * R_CdCm;
  return RigidTransformd(R_WCd, p_WC);
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
  // either the application started listening successfully, or else failed.
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
        params_(params),
        rate_calculator_(params_.realtime_rate_period) {
    DRAKE_THROW_UNLESS(!params.port.has_value() || *params.port == 0 ||
                       *params.port >= 1024);
    if (!drake::internal::IsNetworkingAllowed("meshcat")) {
      throw std::runtime_error(
          "Meshcat has been disabled via the DRAKE_ALLOW_NETWORK environment "
          "variable");
    }

    // Sanity-check the pattern, by passing it (along with dummy host and port
    // values) through to fmt to allow any fmt-specific exception to percolate.
    // Then, confirm that the user's pattern started with a valid protocol.
    const std::string url =
        fmt::format(fmt_runtime(params.web_url_pattern),
                    fmt::arg("host", "foo"), fmt::arg("port", 1));
    if (url.substr(0, 4) != "http") {
      throw std::logic_error("The web_url_pattern must be http:// or https://");
    }

    LoadStaticAssets();

    std::promise<std::tuple<int, bool>> app_promise;
    std::future<std::tuple<int, bool>> app_future = app_promise.get_future();
    websocket_thread_ =
        std::thread(&Impl::WrappedWebSocketMain, this, std::move(app_promise),
                    params.host, params.port);
    bool connected;
    std::tie(port_, connected) = app_future.get();

    if (!connected) {
      mode_.store(kFinished);
      websocket_thread_.join();
      throw std::runtime_error("Meshcat failed to open a websocket port.");
    }

    for (const auto& item : params_.initial_properties) {
      std::visit(
          [this, &item](const auto& value) {
            this->SetProperty(item.path, item.property, value);
          },
          item.value);
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

  // Loads the javascript into CAS. This also serves to cross-check that none of
  // our static resources have gone missing.
  void LoadStaticAssets() {
    // Load the javascript into the CAS cache.
    const std::optional<std::string_view> meshcat_js =
        internal::GetMeshcatStaticResource("/meshcat.js");
    DRAKE_DEMAND(meshcat_js.has_value());
    meshcat_js_ = file_storage_.Insert(std::string{*meshcat_js}, "meshcat.js");

    // Load meshcat.html and rewrite its script citation to use the CAS URL.
    const std::optional<std::string_view> meshcat_html =
        internal::GetMeshcatStaticResource("/meshcat.html");
    DRAKE_DEMAND(meshcat_html.has_value());
    meshcat_html_ = std::string{*meshcat_html};
    const std::string_view old_link{"src=\"meshcat.js\""};
    const size_t start = meshcat_html_.find(old_link);
    DRAKE_DEMAND(start != std::string::npos);
    meshcat_html_.replace(
        start, old_link.size(),
        fmt::format("src=\"{}\"", FileStorage::GetCasUrl(*meshcat_js_)));
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
    return fmt::format(fmt_runtime(params_.web_url_pattern),
                       fmt::arg("host", display_host), fmt::arg("port", port_));
  }

  // This function is public via the PIMPL.
  int port() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    return port_;
  }

  // This function is public via the PIMPL.
  void SetSimulationTime(double sim_time) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    sim_time_ = sim_time;
    const auto report = rate_calculator_.UpdateAndRecalculate(sim_time);
    if (report.initialized) {
      // We won't broadcast it, but we do return to the initialized state.
      realtime_rate_ = 0.0;
      return;
    }
    // This last invocation may have spanned zero or more report periods;
    // dispatch one realtime rate message for each period.
    for (int i = 0; i < report.period_count; ++i) {
      // Note: report.rate may be infinity. The javascript chart will draw a
      // saturated column for that rate value (which will eventually be pushed
      // off the screen) -- but the record of the (smallest, largest) values in
      // the chart will persist in showing infinity, even when the column is no
      // longer visible.
      SetRealtimeRate(report.rate);
    }
  }

  double GetSimulationTime() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    return sim_time_;
  }

  // This function is public via the PIMPL.
  void SetRealtimeRate(double rate) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    realtime_rate_ = rate;
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
  double GetRealtimeRate() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    return realtime_rate_;
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

    internal::SetObjectData data;
    data.path = FullPath(path);

    // TODO(russt): This current meshcat set_object interface couples geometry,
    // material, and object for convenience, but we might consider decoupling
    // them again for efficiency. We don't want to send meshes over the network
    // (which could be from the cloud to a local browser) more than necessary.

    MeshcatShapeReifier reifier(&uuid_generator_, &file_storage_, rgba);
    std::vector<std::shared_ptr<const MemoryFile>> assets;
    MeshcatShapeReifier::Output reifier_output{.lumped = data.object,
                                               .assets = assets};
    shape.Reify(&reifier, &reifier_output);

    if (std::holds_alternative<std::monostate>(data.object.object)) {
      // Then this shape is not supported, and I should not send the message,
      // nor add the object to the tree.
      return;
    }
    if (std::holds_alternative<internal::MeshData>(data.object.object)) {
      auto& meshfile_object = std::get<internal::MeshData>(data.object.object);
      DRAKE_DEMAND(data.object.geometry != nullptr);
      meshfile_object.geometry = data.object.geometry->uuid;

      // Add a material if not already defined.
      if (data.object.material == nullptr) {
        auto material = std::make_unique<internal::MaterialData>();
        material->uuid = uuid_generator_.GenerateRandom();
        material->type = "MeshPhongMaterial";
        material->color = ToMeshcatColor(rgba);
        // TODO(russt): Most values are taken verbatim from meshcat-python.
        material->reflectivity = 0.5;
        material->side = SideOfFaceToRender::kDoubleSide;
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

        meshfile_object.uuid = uuid_generator_.GenerateRandom();
        meshfile_object.material = material->uuid;
        data.object.material = std::move(material);
      }
    }

    Defer([this, data = std::move(data), assets = std::move(assets)]() {
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
      e.object().emplace() = std::move(message);
      e.object()->assets = std::move(assets);
    });
  }

  // This function is public via the PIMPL.
  void SetObject(std::string_view path, const perception::PointCloud& cloud,
                 double point_size, const Rgba& rgba) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->position = cloud.xyzs();
    if (cloud.has_rgbs()) {
      geometry->color = cloud.rgbs().cast<float>() / 255.0;
    }
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuid_generator_.GenerateRandom();
    material->type = "PointsMaterial";
    material->color = ToMeshcatColor(rgba);
    material->transparent = (rgba.a() != 1.0);
    material->opacity = rgba.a();
    material->size = point_size;
    material->vertexColors = cloud.has_rgbs();
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuid_generator_.GenerateRandom();
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
      e.object().emplace() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetObject(std::string_view path, const TriangleSurfaceMesh<double>& mesh,
                 const Rgba& rgba, bool wireframe, double wireframe_line_width,
                 SideOfFaceToRender side) {
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
                    wireframe_line_width, side);
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
    Eigen::Map<Eigen::Matrix3Xd> vertices(vstack.data(), 3, 2 * start.cols());
    const bool kLineSegments = true;
    SetLineImpl(path, vertices, line_width, rgba, kLineSegments);
  }

  // This function is internal to the PIMPL, used to implement the prior two
  // functions (SetLine and SetLineSegments).
  void SetLineImpl(std::string_view path,
                   const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                   double line_width, const Rgba& rgba, bool line_segments) {
    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->position = vertices.cast<float>();
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuid_generator_.GenerateRandom();
    material->type = "LineBasicMaterial";
    material->color = ToMeshcatColor(rgba);
    material->linewidth = line_width;
    material->vertexColors = false;
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuid_generator_.GenerateRandom();
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
      e.object().emplace() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetTriangleMesh(std::string_view path,
                       const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                       const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                       const Rgba& rgba, bool wireframe,
                       double wireframe_line_width, SideOfFaceToRender side) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetObjectData data;
    data.path = FullPath(path);

    SetLumpedObjectFromTriangleMesh(&data.object, vertices, faces, rgba,
                                    wireframe, wireframe_line_width, side,
                                    &uuid_generator_);

    Defer([this, data = std::move(data)]() {
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object().emplace() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetTriangleColorMesh(std::string_view path,
                            const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
                            const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
                            const Eigen::Ref<const Eigen::Matrix3Xd>& colors,
                            bool wireframe, double wireframe_line_width,
                            SideOfFaceToRender side) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::SetObjectData data;
    data.path = FullPath(path);

    auto geometry = std::make_unique<internal::BufferGeometryData>();
    geometry->uuid = uuid_generator_.GenerateRandom();
    geometry->position = vertices.cast<float>();
    geometry->faces = faces.cast<uint32_t>();
    geometry->color = colors.cast<float>();
    data.object.geometry = std::move(geometry);

    auto material = std::make_unique<internal::MaterialData>();
    material->uuid = uuid_generator_.GenerateRandom();
    material->type = "MeshPhongMaterial";
    material->transparent = false;
    material->opacity = 1.0;
    material->wireframe = wireframe;
    material->wireframeLineWidth = wireframe_line_width;
    material->vertexColors = true;
    material->side = side;
    material->flatShading = true;
    data.object.material = std::move(material);

    internal::MeshData mesh;
    mesh.uuid = uuid_generator_.GenerateRandom();
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
      e.object().emplace() = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  template <typename CameraData>
  void SetCamera(CameraData camera, std::string path) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    is_orthographic_ = std::is_same_v<OrthographicCamera, CameraData>;

    internal::SetCameraData<CameraData> data;
    data.path = std::move(path);
    data.object.object = std::move(camera);
    SetCameraTarget({0, 0, 0});

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      e.object().emplace() = std::move(message);
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
      e.transform().emplace() = std::move(message);
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

  // Sets the `property` of `path` to a CAS URL that refers to `file_path`. The
  // file is copied into memory (in `file_storage_`) so that our HTTP server can
  // serve it anytime a browser requests it.
  //
  // This function is a file-internal helper, not public in the PIMPL.
  void SetPropertyToFile(std::string_view path, std::string property,
                         const fs::path& file_path) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    // Read the file and insert it into the database.
    std::optional<std::string> content = ReadFile(file_path);
    if (!content) {
      throw std::runtime_error(fmt::format(
          "Cannot open '{}' when attempting to set property '{}' on '{}'",
          file_path.string(), property, path));
    }
    std::shared_ptr<const MemoryFile> asset =
        file_storage_.Insert(std::move(*content), file_path.string());

    internal::SetPropertyData<std::string> data;
    data.path = FullPath(path);
    data.property = std::move(property);
    data.value = FileStorage::GetCasUrl(*asset);

    Defer([this, data = std::move(data), asset = std::move(asset)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      SceneTreeElement& e = scene_tree_root_[data.path];
      SceneTreeElement::Message& m = e.properties()[data.property];
      m.bytes = std::move(message);
      m.assets = {std::move(asset)};
    });
  }

  // This function is public via the PIMPL.
  DRAKE_NO_EXPORT void SetAnimation(const MeshcatAnimation& animation) {
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

    Defer([this, message = message_stream.str()]() {
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
  // The public version of this only applies to perspective cameras. But, this
  // implementation allows for an override so that SetCameraPose() can orient
  // orthographic cameras as well.
  void SetCameraTarget(const Eigen::Vector3d& p_WT,
                       bool only_perspective = true) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    if (only_perspective && is_orthographic_) return;

    internal::SetCameraTargetData data;
    // The target position in meshcat's y-up world.
    const Eigen::Vector3d p_WT_y = MeshcatYUpPosition(p_WT);
    data.value = {p_WT_y.x(), p_WT_y.y(), p_WT_y.z()};

    Defer([this, data = std::move(data)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      DRAKE_DEMAND(app_ != nullptr);
      std::stringstream message_stream;
      msgpack::pack(message_stream, data);
      std::string message = message_stream.str();
      app_->publish("all", message, uWS::OpCode::BINARY, false);
      camera_target_message_ = std::move(message);
    });
  }

  // This function is public via the PIMPL.
  void SetCameraPose(const Eigen::Vector3d& p_WC, const Eigen::Vector3d& p_WT) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    SetCameraTarget(p_WT, false /* only_perspective */);
    SetTransform("/Cameras/default", math::RigidTransformd());
    // The camera position in meshcat's y-up world.
    const Eigen::Vector3d p_WC_y = MeshcatYUpPosition(p_WC);
    SetProperty("/Cameras/default/rotated/<object>", "position",
                std::vector<double>{p_WC_y.x(), p_WC_y.y(), p_WC_y.z()});
  }

  std::optional<RigidTransformd> GetTrackedCameraPose() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    std::lock_guard<std::mutex> lock(controls_mutex_);
    return camera_pose_;
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
}})))""",
                                data.name);
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
  int GetButtonClicks(std::string_view name) const {
    std::lock_guard<std::mutex> lock(controls_mutex_);
    auto iter = buttons_.find(name);
    if (iter == buttons_.end()) {
      ThrowThingNotFound("button", name, buttons_);
    }
    return iter->second.num_clicks;
  }

  // This function is public via the PIMPL.
  std::vector<std::string> GetButtonNames() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::lock_guard<std::mutex> lock(controls_mutex_);
    std::vector<std::string> names;
    names.reserve(buttons_.size());
    for (const auto& [name, _] : buttons_) {
      names.push_back(name);
    }
    return names;
  }

  // This function is public via the PIMPL.
  bool DeleteButton(std::string name, bool strict) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::DeleteControl data;
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = buttons_.find(name);
      if (iter == buttons_.end()) {
        if (strict) {
          ThrowThingNotFound("button", name, buttons_);
        } else {
          return false;
        }
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
    return true;
  }

  // This function is public via the PIMPL.
  double AddSlider(std::string name, double min, double max, double step,
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
}})))""",
                                data.name);
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
    return value;
  }

  // This function is public via the PIMPL.
  double SetSliderValue(std::string name, double value) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = sliders_.find(name);
      if (iter == sliders_.end()) {
        ThrowThingNotFound("slider", name, sliders_);
      }
      internal::SetSliderControl& s = iter->second;
      // Match setValue in NumberController.js from dat.GUI.
      value = std::max(value, s.min);
      value = std::min(value, s.max);
      value = std::round(value / s.step) * s.step;
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
    return value;
  }

  // This function is public via the PIMPL.
  double GetSliderValue(std::string_view name) const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::lock_guard<std::mutex> lock(controls_mutex_);
    auto iter = sliders_.find(name);
    if (iter == sliders_.end()) {
      ThrowThingNotFound("slider", name, sliders_);
    }
    return iter->second.value;
  }

  // This function is public via the PIMPL.
  std::vector<std::string> GetSliderNames() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::lock_guard<std::mutex> lock(controls_mutex_);
    std::vector<std::string> names;
    names.reserve(sliders_.size());
    for (const auto& [name, _] : sliders_) {
      names.push_back(name);
    }
    return names;
  }

  // This function is public via the PIMPL.
  bool DeleteSlider(std::string name, bool strict) {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    internal::DeleteControl data;
    {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      auto iter = sliders_.find(name);
      if (iter == sliders_.end()) {
        if (strict) {
          ThrowThingNotFound("slider", name, sliders_);
        } else {
          return false;
        }
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
    return true;
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
      DeleteButton(iter->first, /* strict = */ true);
    }
    for (auto iter = sliders.begin(); iter != sliders.end(); ++iter) {
      DeleteSlider(iter->first, /* strict = */ true);
    }
  }

  Meshcat::Gamepad GetGamepad() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));

    std::lock_guard<std::mutex> lock(controls_mutex_);
    return gamepad_;
  }

  // This function is for use by the websocket thread. The Meshcat::StaticHtml()
  // and Meshcat::StaticZip() outer functions call into here using appropriate
  // deferred handling.
  std::string CalcStandaloneHtml(bool zip) const {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));

    // Bundle the js file.
    // For zip, we'll link to the CAS URL.
    // For non-zip, we insert the javascript into the html.
    std::string html;
    std::unique_ptr<MeshcatZipFactory> zip_factory;
    if (zip) {
      html = meshcat_html_;
      zip_factory = std::make_unique<MeshcatZipFactory>();
    } else {
      html = internal::GetMeshcatStaticResource("/meshcat.html").value();
      const std::string_view src_link{" src=\"meshcat.js\""};
      const size_t js_pos = html.find(src_link);
      DRAKE_DEMAND(js_pos != std::string::npos);
      html.erase(js_pos, src_link.size());
      html.insert(js_pos + 1, meshcat_js_->contents());
    }

    // Add the assets. For zip, we add the assets to the archive.
    // For non-zip, we encode the assets into the html and insert a JavaScript
    // URL hook that knows how to serve the CAS database.
    // (See FileStorage and GetCasUrl for details about CAS.)
    std::string javascript;
    if (zip) {
      std::vector<std::shared_ptr<const MemoryFile>> assets =
          file_storage_.DumpEverything();
      for (const auto& asset : assets) {
        const std::string asset_url = FileStorage::GetCasUrl(*asset);
        zip_factory->AddFile(asset_url, asset->contents());
      }
    } else {
      javascript = "casAssets = {};\n";
      std::vector<std::shared_ptr<const MemoryFile>> assets =
          file_storage_.DumpEverything();
      for (const auto& asset : assets) {
        if (asset->sha256() == meshcat_js_->sha256()) {
          // We already inserted this resource directly into the html above, so
          // there's no need to dump it as part of the CAS.
          continue;
        }
        javascript += fmt::format("// {}\n", asset->filename_hint());
        javascript += fmt::format(
            "casAssets[\"{}\"] = "
            "\"data:application/octet-binary;base64,{}\";\n",
            FileStorage::GetCasUrl(*asset),
            common_robotics_utilities::base64_helpers::Encode(
                std::vector<uint8_t>(asset->contents().begin(),
                                     asset->contents().end())));
      }
      javascript += R"""(
          MeshCat.THREE.DefaultLoadingManager.setURLModifier(url => {
              if (url in casAssets) {
                  return casAssets[url];
              }
              return url;
          });
)""";
    }

    // Replace the javascript code in the original html file which connects via
    // websockets with the static javascript commands.
    javascript += scene_tree_root_.CreateCommands();
    if (!animation_.empty()) {
      javascript += CreateCommand(animation_);
    }
    if (!camera_target_message_.empty()) {
      javascript += CreateCommand(camera_target_message_);
    }
    std::regex block_re(
        "<!-- CONNECTION BLOCK BEGIN [^]+ CONNECTION BLOCK END -->\n");
    html = std::regex_replace(html, block_re, javascript);

    if (zip) {
      zip_factory->AddFile("meshcat.html", std::move(html));
      return zip_factory->Build();
    } else {
      return html;
    }
  }

  // This function is public via the PIMPL.
  std::string StaticHtml() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    Defer([this, p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      p.set_value(CalcStandaloneHtml(/* zip = */ false));
    });
    return f.get();
  }

  // This function is public via the PIMPL.
  std::string StaticZip() const {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    std::promise<std::string> p;
    std::future<std::string> f = p.get_future();
    Defer([this, p = std::move(p)]() mutable {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      p.set_value(CalcStandaloneHtml(/* zip = */ true));
    });
    return f.get();
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
      std::string value;
      if ((e != nullptr) && e->object().has_value()) {
        value = e->object().value().bytes;
      }
      p.set_value(std::move(value));
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
      std::string value;
      if ((e != nullptr) && e->transform().has_value()) {
        value = e->transform().value().bytes;
      }
      p.set_value(std::move(value));
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
      std::string value;
      if (e != nullptr) {
        auto prop = e->properties().find(property);
        if (prop != e->properties().end()) {
          value = prop->second.bytes;
        }
      }
      p.set_value(std::move(value));
    });
    return f.get();
  }

  void InjectWebsocketMessage(std::string_view message) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    std::promise<void> promise;
    std::future<void> future = promise.get_future();
    Defer([this, &promise, message_copy = std::string(message)]() {
      DRAKE_DEMAND(IsThread(websocket_thread_id_));
      this->HandleMessage(/* ws = */ nullptr, message_copy);
      promise.set_value();
    });
    future.wait();
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

  void InjectMockTimer(std::unique_ptr<Timer> timer) {
    DRAKE_DEMAND(IsThread(main_thread_id_));
    rate_calculator_.InjectMockTimer(std::move(timer));
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
  void WrappedWebSocketMain(std::promise<std::tuple<int, bool>> app_promise,
                            std::string host, std::optional<int> desired_port) {
    try {
      WebSocketMain(std::move(app_promise), host, desired_port);
    } catch (const std::exception& e) {
      drake::log()->critical(
          "Meshcat's internal websocket thread crashed via an exception: {}",
          e.what());
    }
  }

  void WebSocketMain(std::promise<std::tuple<int, bool>> app_promise,
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

    uWS::App::WebSocketBehavior<PerSocketData> behavior;
    // Set maxBackpressure = 0 so that uWS does *not* drop any messages due to
    // back pressure.
    behavior.maxBackpressure = 0;
    behavior.open = [this](WebSocket* ws) {
      // IsThread(websocket_thread_id_) is checked by the Handle... function.
      HandleSocketOpen(ws);
    };
    behavior.close = [this](WebSocket* ws, int, std::string_view) {
      // IsThread(websocket_thread_id_) is checked by the Handle... function.
      HandleSocketClose(ws);
    };
    behavior.message = [this](WebSocket* ws, std::string_view message,
                              uWS::OpCode) {
      // IsThread(websocket_thread_id_) is checked by the Handle... function.
      HandleMessage(ws, message);
    };

    uWS::App app = uWS::App()
                       .get("/*",
                            [this](uWS::HttpResponse<kSsl>* response,
                                   uWS::HttpRequest* request) {
                              DRAKE_DEMAND(IsThread(websocket_thread_id_));
                              this->HandleHttpGet(request->getUrl(), response);
                            })
                       .ws<PerSocketData>("/*", std::move(behavior));
    app_ = &app;

    // Search for an open port.
    int chosen_port{};
    const int search_start = desired_port.value_or(7000);
    const int search_end = desired_port.value_or(8000);
    for (int port = search_start; port <= search_end; ++port) {
      // N.B. Using `port == 0` requests an ephemeral port.
      // https://github.com/uNetworking/uSockets/pull/136.
      app.listen(bind_host, port, LIBUS_LISTEN_EXCLUSIVE_PORT,
                 [this](us_listen_socket_t* socket) {
                   DRAKE_DEMAND(IsThread(websocket_thread_id_));
                   if (socket) {
                     listen_socket_ = socket;
                   }
                 });
      if (listen_socket_ != nullptr) {
        chosen_port = us_socket_local_port(
            /* ssl = */ 0, reinterpret_cast<us_socket_t*>(listen_socket_));
        DRAKE_THROW_UNLESS(chosen_port > 0);
        break;
      }
    }

    bool connected = listen_socket_ != nullptr;
    app_promise.set_value(std::make_tuple(chosen_port, connected));
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
    drake::log()->debug("Meshcat connection opened from {}",
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

    if (!camera_target_message_.empty()) {
      ws->send(camera_target_message_);
    }

    // Tell client if the realtime rate plot should be hidden
    internal::ShowRealtimeRate realtime_rate_message;
    realtime_rate_message.show = params_.show_stats_plot;
    std::stringstream realtime_message_stream;
    msgpack::pack(realtime_message_stream, realtime_rate_message);
    ws->send(realtime_message_stream.str());

    if (inject_open_fault_.load()) {
      throw std::runtime_error("InjectWebsocketThreadFault during socket open");
    }
  }

  // This function is a callback from a WebSocketBehavior.
  void HandleSocketClose(WebSocket* ws) {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));
    drake::log()->debug("Meshcat connection closed from {}",
                        ws->getRemoteAddressAsText());
    websockets_.erase(ws);
    const int new_count = --num_websockets_;
    DRAKE_DEMAND(new_count >= 0);
    DRAKE_DEMAND(new_count == static_cast<int>(websockets_.size()));
    if (ws == camera_pose_source_) {
      std::lock_guard<std::mutex> lock(controls_mutex_);
      camera_pose_source_ = nullptr;
      camera_pose_ = std::nullopt;
    }
  }

  // This function is called from uWS when it needs to service a GET request.
  void HandleHttpGet(std::string_view url_path,
                     uWS::HttpResponse<kSsl>* response) const {
    DRAKE_DEMAND(IsThread(websocket_thread_id_));
    drake::log()->debug("Meshcat: GET {}", url_path);
    // Handle the magic download URLs.
    if (url_path == "/download") {
      const std::string content = CalcStandaloneHtml(/* zip = */ false);
      response->writeHeader("Content-Type", "text/html; charset=utf-8");
      response->writeHeader("Content-Disposition",
                            "attachment; filename=\"meshcat.html\"");
      response->end(content);
      return;
    }
    if (url_path == "/download.zip") {
      const std::string content = CalcStandaloneHtml(/* zip = */ true);
      response->writeHeader("Content-Type", "application/x-zip");
      response->writeHeader("Content-Disposition",
                            "attachment; filename=\"meshcat.zip\"");
      response->end(content);
      return;
    }
    // Handle content-addressable storage. This must align with GetCasUrl() in
    // FileStorage so if you change it be sure to change both places.
    if (url_path.substr(0, 8) == "/cas-v1/") {
      const std::string_view suffix = url_path.substr(8);
      std::optional<Sha256> key = Sha256::Parse(suffix);
      if (!key.has_value()) {
        drake::log()->warn("Meshcat: Malformed CAS key {}", suffix);
        response->writeStatus("400 Unparseable CAS key");
        response->end("");
        return;
      }
      std::shared_ptr<const MemoryFile> handle = file_storage_.Find(*key);
      if (handle == nullptr) {
        drake::log()->warn(
            "Meshcat: Unknown CAS key {} (there are {} assets in the cache)",
            suffix, file_storage_.size());
        response->writeStatus("404 CAS key not found");
        response->writeHeader("Cache-Control", "no-cache");
        response->end("");
        return;
      }
      response->writeHeader("Meshcat-Cas-Filename", handle->filename_hint());
      // https://developer.mozilla.org/en-US/docs/Web/HTTP/Headers/Cache-Control#immutable
      response->writeHeader("Cache-Control",
                            "public, max-age=604800, immutable");
      response->end(handle->contents());
      return;
    }
    // Handle static (i.e., compiled-in) files.
    if ((url_path == "/") || (url_path == "/index.html") ||
        (url_path == "/meshcat.html")) {
      response->writeHeader("Content-Type", "text/html; charset=utf-8");
      response->writeHeader("Cache-Control", "no-cache");
      response->end(meshcat_html_);
      return;
    }
    if (url_path == "/favicon.ico") {
      response->end(internal::GetMeshcatStaticResource(url_path).value());
      return;
    }
    // Unknown URL.
    log()->warn("Meshcat: Failed http request for unknown URL {}", url_path);
    response->writeStatus("404 Not Found");
    response->writeHeader("Cache-Control", "no-cache");
    response->end("");
  }

  // This function is a callback from a WebSocketBehavior. However, unit tests
  // may also call it (via InjectWebsocketMessage) in which case the `ws` will
  // be null.
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
      drake::log()->debug("Meshcat ignored an unparsable message");
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
          DRAKE_DEMAND(ws != nullptr);
          ws->publish("all", message_stream.str(), uWS::OpCode::BINARY);
        }
      }
      return;
    }
    if (data.type == "gamepad" && data.gamepad.has_value()) {
      // TODO(russt): Figure out how to do the non-invasive unpack of
      // Meshcat::Gamepad and remove internal::Gamepad.
      gamepad_.index = data.gamepad->index;
      gamepad_.button_values = std::move(data.gamepad->button_values);
      gamepad_.axes = std::move(data.gamepad->axes);
      return;
    }
    if (data.type == "camera_pose" && data.camera_pose.size() == 16 &&
        data.is_perspective.has_value()) {
      if (camera_pose_source_ != nullptr && camera_pose_source_ != ws) {
        static const logging::Warn log_once(
            "More than one meshcat client is attempting to broadcast its "
            "camera pose. The view rendered will be that of the browser whose "
            "camera was last modified.");
      }
      camera_pose_source_ = ws;
      if (*data.is_perspective) {
        camera_pose_ =
            MakeDrakePoseFromMeshcatPoseForCamera(std::move(data.camera_pose));
      } else {
        camera_pose_ = std::nullopt;
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

  // Both threads access the following variables, guarded by controls_mutex_.
  mutable std::mutex controls_mutex_;
  std::map<std::string, internal::SetButtonControl, std::less<>> buttons_{};
  std::map<std::string, internal::SetSliderControl, std::less<>> sliders_{};
  std::string camera_target_message_;
  Meshcat::Gamepad gamepad_{};
  std::vector<std::string> controls_{};  // Names of buttons and sliders in the
                                         // order they were added.
  // The socket for the browser that is sending the camera pose.
  WebSocket* camera_pose_source_{};
  std::optional<math::RigidTransformd> camera_pose_;

  // These variables should only be accessed in the main thread, where "main
  // thread" is the thread in which this class was constructed.
  std::thread::id main_thread_id_{};
  const MeshcatParams params_;
  int port_{};
  internal::UuidGenerator uuid_generator_{};
  double sim_time_{};
  systems::internal::RealtimeRateCalculator rate_calculator_;
  double realtime_rate_{0.0};
  bool is_orthographic_{false};

  // These variables should only be accessed in the websocket thread.
  std::thread::id websocket_thread_id_{};
  SceneTreeElement scene_tree_root_{};
  std::string animation_;
  uWS::App* app_{nullptr};
  us_listen_socket_t* listen_socket_{nullptr};
  std::set<WebSocket*> websockets_{};
  std::shared_ptr<const MemoryFile> meshcat_js_;
  std::string meshcat_html_;

  // This variable may be accessed from any thread, but should only be modified
  // in the websocket thread.
  std::atomic<int> num_websockets_{0};

  // This variable may be accessed from any thread. It has an internal mutex.
  FileStorage file_storage_;

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

Meshcat::Meshcat(std::optional<int> port)
    : Meshcat(MeshcatParams{.port = port}) {}

Meshcat::Meshcat(const MeshcatParams& params)
    // The Impl constructor creates the server thread, binds to the port, etc.
    : impl_{new Impl(params)},
      recording_{std::make_unique<internal::MeshcatRecording>()} {
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
                        double wireframe_line_width, SideOfFaceToRender side) {
  impl().SetObject(path, mesh, rgba, wireframe, wireframe_line_width, side);
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
    bool wireframe, double wireframe_line_width, SideOfFaceToRender side) {
  impl().SetTriangleMesh(path, vertices, faces, rgba, wireframe,
                         wireframe_line_width, side);
}

void Meshcat::SetTriangleColorMesh(
    std::string_view path, const Eigen::Ref<const Eigen::Matrix3Xd>& vertices,
    const Eigen::Ref<const Eigen::Matrix3Xi>& faces,
    const Eigen::Ref<const Eigen::Matrix3Xd>& colors, bool wireframe,
    double wireframe_line_width, SideOfFaceToRender side) {
  impl().SetTriangleColorMesh(path, vertices, faces, colors, wireframe,
                              wireframe_line_width, side);
}

void Meshcat::PlotSurface(std::string_view path,
                          const Eigen::Ref<const Eigen::MatrixXd>& X,
                          const Eigen::Ref<const Eigen::MatrixXd>& Y,
                          const Eigen::Ref<const Eigen::MatrixXd>& Z,
                          const Rgba& rgba, bool wireframe,
                          double wireframe_line_width) {
  DRAKE_DEMAND(Y.rows() == X.rows() && Y.cols() == X.cols());
  DRAKE_DEMAND(Z.rows() == X.rows() && Z.cols() == X.cols());
  const int rows = X.rows(), cols = X.cols();

  if (wireframe) {
    int count = -1;
    Eigen::Matrix3Xd vertices(3, rows * cols * 2);
    // Sweep back and forth along rows.
    for (int r = 0; r < rows; ++r) {
      const int c0 = (r & 0x1) ? cols - 1 : 0;
      const int c_delta = (r & 0x1) ? -1 : 1;
      for (int j = 0, c = c0; j < cols; ++j, c += c_delta) {
        vertices.col(++count) << X(r, c), Y(r, c), Z(r, c);
      }
    }
    // Sweep back and forth along columns.
    const int c0 = (rows & 0x1) ? cols - 1 : 0;
    const int c_delta = (rows & 0x1) ? -1 : 1;
    for (int j = 0, c = c0; j < cols; ++j, c += c_delta) {
      const int r0 = (j & 0x1) ? 0 : rows - 1;
      const int r_delta = (j & 0x1) ? 1 : -1;
      for (int i = 0, r = r0; i < rows; ++i, r += r_delta) {
        vertices.col(++count) << X(r, c), Y(r, c), Z(r, c);
      }
    }

    impl().SetLine(path, vertices, wireframe_line_width, rgba);
  } else {
    using MapRowVector = const Eigen::Map<const Eigen::RowVectorXd>;

    Eigen::Matrix3Xd vertices(3, rows * cols);
    vertices.row(0) = MapRowVector(X.data(), rows * cols);
    vertices.row(1) = MapRowVector(Y.data(), rows * cols);
    vertices.row(2) = MapRowVector(Z.data(), rows * cols);

    // Make a regular grid as in https://stackoverflow.com/q/44934631.
    const int num_boxes = (rows - 1) * (cols - 1);
    Eigen::Matrix3Xi faces(3, 2 * num_boxes);
    Eigen::MatrixXi ids(rows, cols);
    // Populate ids with [0, 1, ..., num vertices-1]
    std::iota(ids.data(), ids.data() + rows * cols, 0);

    int count = 0;
    for (int i = 0; i < rows - 1; ++i) {
      for (int j = 0; j < cols - 1; ++j) {
        // Upper left triangles.
        faces.col(count++) << ids(i, j), ids(i + 1, j), ids(i, j + 1);
        // Lower right triangles.
        faces.col(count++) << ids(i + 1, j), ids(i + 1, j + 1), ids(i, j + 1);
      }
    }

    impl().SetTriangleMesh(path, vertices, faces, rgba, wireframe,
                           wireframe_line_width,
                           SideOfFaceToRender::kDoubleSide);
  }
}

void Meshcat::SetCamera(PerspectiveCamera camera, std::string path) {
  impl().SetCamera(std::move(camera), std::move(path));
}

void Meshcat::SetCamera(OrthographicCamera camera, std::string path) {
  impl().SetCamera(std::move(camera), std::move(path));
}

void Meshcat::SetTransform(std::string_view path,
                           const RigidTransformd& X_ParentPath,
                           std::optional<double> time_in_recording) {
  const bool show_live =
      recording_->SetTransform(path, X_ParentPath, time_in_recording);
  if (show_live) {
    impl().SetTransform(path, X_ParentPath);
  }
}

void Meshcat::SetTransform(std::string_view path,
                           const Eigen::Ref<const Eigen::Matrix4d>& matrix) {
  impl().SetTransform(path, matrix);
}

void Meshcat::Delete(std::string_view path) {
  impl().Delete(path);
}

void Meshcat::SetSimulationTime(double sim_time) {
  impl().SetSimulationTime(sim_time);
}

double Meshcat::GetSimulationTime() const {
  return impl().GetSimulationTime();
}

void Meshcat::SetRealtimeRate(double rate) {
  impl().SetRealtimeRate(rate);
}

double Meshcat::GetRealtimeRate() const {
  return impl().GetRealtimeRate();
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          bool value, std::optional<double> time_in_recording) {
  const bool show_live =
      recording_->SetProperty(path, property, value, time_in_recording);
  if (show_live) {
    impl().SetProperty(path, std::move(property), value);
  }
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          double value,
                          std::optional<double> time_in_recording) {
  const bool show_live =
      recording_->SetProperty(path, property, value, time_in_recording);
  if (show_live) {
    impl().SetProperty(path, std::move(property), value);
  }
}

void Meshcat::SetProperty(std::string_view path, std::string property,
                          const std::vector<double>& value,
                          std::optional<double> time_in_recording) {
  const bool show_live =
      recording_->SetProperty(path, property, value, time_in_recording);
  if (show_live) {
    impl().SetProperty(path, std::move(property), value);
  }
}

void Meshcat::SetEnvironmentMap(const fs::path& image_path) {
  const std::string_view property_path = "/Background/<object>";
  const std::string property_name = "environment_map";
  if (image_path.empty()) {
    impl().SetProperty(property_path, property_name, std::string{});
  } else {
    impl().SetPropertyToFile(property_path, property_name, image_path);
  }
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

void Meshcat::SetCameraTarget(const Eigen::Vector3d& target_in_world) {
  impl().SetCameraTarget(target_in_world);
}

void Meshcat::SetCameraPose(const Eigen::Vector3d& camera_in_world,
                            const Eigen::Vector3d& target_in_world) {
  impl().SetCameraPose(camera_in_world, target_in_world);
}

std::optional<RigidTransformd> Meshcat::GetTrackedCameraPose() const {
  return impl().GetTrackedCameraPose();
}

void Meshcat::AddButton(std::string name, std::string keycode) {
  impl().AddButton(std::move(name), std::move(keycode));
}

int Meshcat::GetButtonClicks(std::string_view name) const {
  return impl().GetButtonClicks(name);
}

std::vector<std::string> Meshcat::GetButtonNames() const {
  return impl().GetButtonNames();
}

bool Meshcat::DeleteButton(std::string name, bool strict) {
  return impl().DeleteButton(std::move(name), strict);
}

double Meshcat::AddSlider(std::string name, double min, double max, double step,
                          double value, std::string decrement_keycode,
                          std::string increment_keycode) {
  return impl().AddSlider(std::move(name), min, max, step, value,
                          std::move(decrement_keycode),
                          std::move(increment_keycode));
}

double Meshcat::SetSliderValue(std::string name, double value) {
  return impl().SetSliderValue(std::move(name), value);
}

double Meshcat::GetSliderValue(std::string_view name) const {
  return impl().GetSliderValue(name);
}

std::vector<std::string> Meshcat::GetSliderNames() const {
  return impl().GetSliderNames();
}

bool Meshcat::DeleteSlider(std::string name, bool strict) {
  return impl().DeleteSlider(std::move(name), strict);
}

void Meshcat::DeleteAddedControls() {
  impl().DeleteAddedControls();
}

Meshcat::Gamepad Meshcat::GetGamepad() const {
  return impl().GetGamepad();
}

std::string Meshcat::StaticHtml() const {
  return impl().StaticHtml();
}

std::string Meshcat::StaticZip() const {
  return impl().StaticZip();
}

void Meshcat::StartRecording(double frames_per_second,
                             bool set_visualizations_while_recording) {
  recording_->StartRecording(frames_per_second,
                             set_visualizations_while_recording);
}

void Meshcat::StopRecording() {
  recording_->StopRecording();
}

void Meshcat::PublishRecording() {
  impl().SetAnimation(recording_->get_animation());
}

void Meshcat::DeleteRecording() {
  recording_->DeleteRecording();
}

const MeshcatAnimation& Meshcat::get_recording() const {
  return recording_->get_animation();
}

MeshcatAnimation& Meshcat::get_mutable_recording() {
  return recording_->get_mutable_animation();
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

void Meshcat::InjectWebsocketMessage(std::string_view message) {
  impl().InjectWebsocketMessage(message);
}

void Meshcat::InjectWebsocketThreadFault(int fault_number) {
  impl().InjectWebsocketThreadFault(fault_number);
}

void Meshcat::InjectMockTimer(std::unique_ptr<Timer> timer) {
  impl().InjectMockTimer(std::move(timer));
}

}  // namespace geometry
}  // namespace drake
