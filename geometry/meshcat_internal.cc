#include "drake/geometry/meshcat_internal.h"

#include <functional>
#include <stdexcept>
#include <utility>

#include <common_robotics_utilities/base64_helpers.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <uuid.h>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_export.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
#include "drake/common/overloaded.h"
#include "drake/common/text_logging.h"

namespace drake {
namespace geometry {
namespace internal {

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

std::string LoadResource(const std::string& resource_name) {
  const std::string resource = FindResourceOrThrow(resource_name);
  std::optional<std::string> content = ReadFile(resource);
  if (!content) {
    throw std::runtime_error(
        fmt::format("Error opening resource: {}", resource_name));
  }
  return std::move(*content);
}

}  //  namespace

std::optional<std::string_view> GetMeshcatStaticResource(
    std::string_view url_path) {
  static const drake::never_destroyed<std::string> meshcat_js(
      LoadResource("drake/geometry/meshcat.js"));
  static const drake::never_destroyed<std::string> meshcat_ico(
      LoadResource("drake/geometry/meshcat.ico"));
  static const drake::never_destroyed<std::string> meshcat_html(
      LoadResource("drake/geometry/meshcat.html"));
  if (url_path == "/meshcat.html") {
    return meshcat_html.access();
  }
  if (url_path == "/meshcat.js") {
    return meshcat_js.access();
  }
  if (url_path == "/favicon.ico") {
    return meshcat_ico.access();
  }
  return {};
}

// We need an Impl for this class for two reasons:
// - The mt19937 object is ginormous and should not be inline.
// - The uuids object must be wrapped within DRAKE_NO_EXPORT.
struct DRAKE_NO_EXPORT UuidGenerator::Impl {
  std::mt19937 prng_;
  uuids::uuid_random_generator uuid_{prng_};
};

UuidGenerator::UuidGenerator() : impl_(std::make_unique<Impl>()) {}

UuidGenerator::~UuidGenerator() = default;

std::string UuidGenerator::GenerateRandom() {
  return uuids::to_string(impl_->uuid_());
}

namespace {

// The result of loading a URI: the URI's contents (maybe) and a description.
struct UriLoadResult {
  // The contents of a requested Uri. May be null if the URI could not be
  // successfully read.
  std::optional<std::string> contents;

  // A uri-loader-dependent description of the URI to use in messaging to the
  // user.
  std::string description;
};

// The function that attempts to load a URI.
using UriLoader = std::function<UriLoadResult(std::string_view)>;

// Load the given `uri` into `storage` and returns the storage handle.
// On error, returns nullptr.
// @param array_hint The place where this URI occurred, e.g., "buffers[1]".
//  Only used for debugging and error messages.
// @param uri_loader The function that attempts to read the file contents
//  associated with the given `uri`.
// @param description User-friendly string describing the glTF file the URI
//  comes from. Only used for debugging and error messages.
std::shared_ptr<const MemoryFile> LoadGltfUri(std::string_view array_hint,
                                              std::string_view uri,
                                              FileStorage* storage,
                                              const UriLoader& uri_loader,
                                              const std::string& description) {
  DRAKE_DEMAND(storage != nullptr);
  if (uri.substr(0, 5) == "data:") {
    constexpr std::string_view needle = ";base64,";
    const size_t pos = uri.find(needle);
    if (pos == std::string_view::npos) {
      log()->warn(
          "Meshcat ignoring malformed data URI while loading {} from '{}'",
          array_hint, description);
      return nullptr;
    }
    // TODO(jwnimmer-tri) Save the media type to http-serve later on.
    // For now, we'll just skip ahead to the actual content. Note: for a data
    // URI, glTF requires the mime type be specified. It's optional for a file
    // URI and, if absent, we'd have to rely on the file URI's extension.
    std::string_view base64_content = uri.substr(pos + needle.size());
    // TODO(jwnimmer-tri) Use a decoder with a better types, to avoid all of
    // these extra copies.
    const std::vector<uint8_t> decoded_vec =
        common_robotics_utilities::base64_helpers::Decode(
            std::string{base64_content});
    std::string decoded(reinterpret_cast<const char*>(decoded_vec.data()),
                        decoded_vec.size());
    std::string filename_hint =
        fmt::format("{} from {}", array_hint, description);
    return storage->Insert(std::move(decoded), std::move(filename_hint));
  } else {
    // Not a data URI, so it must be a relative path.
    UriLoadResult asset_data = uri_loader(uri);
    if (!asset_data.contents.has_value()) {
      log()->warn(
          "Meshcat could not get data for the named uri '{}' while loading {} "
          "from '{}'",
          uri, array_hint, description);
      return nullptr;
    }
    return storage->Insert(std::move(*asset_data.contents),
                           std::move(asset_data.description));
  }
}

}  // namespace

std::vector<std::shared_ptr<const MemoryFile>> UnbundleGltfAssets(
    const MeshSource& mesh_source, std::string* gltf_contents,
    FileStorage* storage) {
  DRAKE_DEMAND(gltf_contents != nullptr);
  DRAKE_DEMAND(storage != nullptr);
  std::vector<std::shared_ptr<const MemoryFile>> assets;

  json gltf;
  try {
    gltf = json::parse(*gltf_contents);
  } catch (const json::exception& e) {
    log()->warn("Meshcat could not unbundle '{}': glTF parse error: {}",
                mesh_source.description(), e.what());
    return assets;
  }

  // Resolving URIs depends on where the glTF specification resides.
  UriLoader uri_loader;
  if (mesh_source.is_path()) {
    uri_loader = [gltf_dir =
                      mesh_source.path().parent_path()](std::string_view uri) {
      const fs::path uri_path = gltf_dir / uri;
      return UriLoadResult{ReadFile(uri_path), uri_path.string()};
    };
  } else {
    DRAKE_DEMAND(mesh_source.is_in_memory());
    uri_loader = [&memory_mesh = mesh_source.in_memory()](
                     std::string_view uri) -> UriLoadResult {
      const std::string description =
          fmt::format("glTF '{}' supporting file with uri '{}'",
                      memory_mesh.mesh_file.filename_hint(), uri);
      std::optional<std::string> contents;

      const auto file_source_iter = memory_mesh.supporting_files.find(uri);
      if (file_source_iter != memory_mesh.supporting_files.end()) {
        contents = std::visit<std::optional<std::string>>(
            overloaded{[](const fs::path& path) {
                         // Either uri is absolute or meaningful w.r.t. cwd,
                         // otherwise, we'll respond appropriately to an
                         // otherwise unavailable uri.
                         return ReadFile(path);
                       },
                       [](const MemoryFile& file) {
                         return file.contents();
                       }},
            file_source_iter->second);
      }

      return {std::move(contents), description};
    };
  }

  // In glTF 2.0, URIs can only appear in two places:
  //  "images": [ { "uri": "some.png" } ]
  //  "buffers": [ { "uri": "some.bin", "byteLength": 1024 } ]
  bool edited = false;
  for (const auto& array_name : {"images", "buffers"}) {
    auto& array = gltf[array_name];
    for (size_t i = 0; i < array.size(); ++i) {
      auto& item = array[i];
      if (item.contains("uri") && item["uri"].is_string()) {
        const std::string_view uri =
            item["uri"].template get<std::string_view>();
        const std::string array_hint = fmt::format("{}[{}]", array_name, i);
        std::shared_ptr<const MemoryFile> asset = LoadGltfUri(
            array_hint, uri, storage, uri_loader, mesh_source.description());
        if (asset != nullptr) {
          item["uri"] = FileStorage::GetCasUrl(*asset);
          assets.push_back(std::move(asset));
          edited = true;
        }
      }
    }
  }
  if (edited) {
    *gltf_contents = gltf.dump();
  }
  return assets;
}

template <typename T>
std::string TransformGeometryName(GeometryId geom_id,
                                  const SceneGraphInspector<T>& inspector) {
  std::string geometry_name = inspector.GetName(geom_id);
  size_t pos = 0;
  while ((pos = geometry_name.find("::", pos)) != std::string::npos) {
    geometry_name.replace(pos++, 2, "/");
  }
  return geometry_name;
}

DRAKE_DEFINE_FUNCTION_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    (&TransformGeometryName<T>));

}  // namespace internal
}  // namespace geometry
}  // namespace drake
