#include "drake/geometry/meshcat_internal.h"

#include <stdexcept>
#include <utility>

#include <common_robotics_utilities/base64_helpers.hpp>
#include <fmt/format.h>
#include <nlohmann/json.hpp>
#include <uuid.h>

#include "drake/common/drake_export.h"
#include "drake/common/find_resource.h"
#include "drake/common/never_destroyed.h"
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
  static const drake::never_destroyed<std::string> stats_js(
      LoadResource("drake/geometry/stats.min.js"));
  static const drake::never_destroyed<std::string> meshcat_ico(
      LoadResource("drake/geometry/meshcat.ico"));
  static const drake::never_destroyed<std::string> meshcat_html(
      LoadResource("drake/geometry/meshcat.html"));
  if ((url_path == "/") || (url_path == "/index.html") ||
      (url_path == "/meshcat.html")) {
    return meshcat_html.access();
  }
  if (url_path == "/meshcat.js") {
    return meshcat_js.access();
  }
  if (url_path == "/stats.min.js") {
    return stats_js.access();
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

// Load the given `uri` into `storage` and returns the storage handle.
// On error, returns nullptr.
// @param gltf_filename Only used for debugging and error messages.
// @param array_hint The place where this URI occurred, e.g., "buffers[1]".
//  Only used for debugging and error messages.
std::shared_ptr<const FileStorage::Handle> LoadGltfUri(
    const fs::path& gltf_filename, std::string_view array_hint,
    std::string_view uri, FileStorage* storage) {
  DRAKE_DEMAND(storage != nullptr);
  if (uri.substr(0, 5) == "data:") {
    constexpr std::string_view needle = ";base64,";
    const size_t pos = uri.find(needle);
    if (pos == std::string_view::npos) {
      log()->warn(
          "Meshcat ignoring malformed data URI while loading {} from '{}'",
          array_hint, gltf_filename.string());
      return nullptr;
    }
    // TODO(jwnimmer-tri) Save the media type to http-serve later on.
    // For now, we'll just skip ahead to the actual content.
    std::string_view base64_content = uri.substr(pos + needle.size());
    // TODO(jwnimmer-tri) Use a decoder with a better types, to avoid all of
    // these extra copies.
    const std::vector<uint8_t> decoded_vec =
        common_robotics_utilities::base64_helpers::Decode(
            std::string{base64_content});
    std::string decoded(reinterpret_cast<const char*>(decoded_vec.data()),
                        decoded_vec.size());
    std::string filename_hint =
        fmt::format("{} from {}", array_hint, gltf_filename.string());
    return storage->Insert(std::move(decoded), std::move(filename_hint));
  } else {
    // Not a data URI, so it must be a relative path.
    fs::path asset_filename = gltf_filename.parent_path() / uri;
    std::optional<std::string> asset_data = ReadFile(asset_filename);
    if (!asset_data) {
      log()->warn(
          "Meshcat could not open '{}' while loading {} from '{}'",
          asset_filename.string(), array_hint, gltf_filename.string());
      return nullptr;
    }
    return storage->Insert(std::move(*asset_data), asset_filename.string());
  }
}

}  // namespace

std::vector<std::shared_ptr<const FileStorage::Handle>> UnbundleGltfAssets(
    const fs::path& gltf_filename, std::string* gltf_contents,
    FileStorage* storage) {
  DRAKE_DEMAND(gltf_contents != nullptr);
  DRAKE_DEMAND(storage != nullptr);
  std::vector<std::shared_ptr<const FileStorage::Handle>> assets;
  json gltf;
  try {
    gltf = json::parse(*gltf_contents);
  } catch (const json::exception& e) {
    log()->warn("Meshcat could not unbundle '{}': glTF parse error: {}",
                gltf_filename.string(), e.what());
    return assets;
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
        std::shared_ptr<const FileStorage::Handle> asset =
            LoadGltfUri(gltf_filename, array_hint, uri, storage);
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

}  // namespace internal
}  // namespace geometry
}  // namespace drake
