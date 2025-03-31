#include "drake/geometry/read_gltf_to_memory.h"

#include <string>
#include <utility>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "drake/common/file_source.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {

using nlohmann::json;

namespace {

void AddFilesFromUris(const std::filesystem::path& gltf_path, const json& gltf,
                      std::string_view array_name,
                      string_map<FileSource>* supporting_files) {
  if (!gltf.contains(array_name)) return;
  auto& array = gltf[array_name];
  for (size_t i = 0; i < array.size(); ++i) {
    auto& item = array[i];
    if (item.contains("uri") && item["uri"].is_string()) {
      const std::string_view uri = item["uri"].template get<std::string_view>();
      if (uri.substr(0, 5) == "data:") {
        continue;
      }
      supporting_files->emplace(uri, gltf_path.parent_path() / uri);
    }
  }
}

}  // namespace

InMemoryMesh ReadGltfToMemory(const std::filesystem::path& gltf_path) {
  auto gltf_file = MemoryFile::Make(gltf_path);

  string_map<FileSource> supporting_files;
  json gltf;
  try {
    gltf = json::parse(gltf_file.contents());
  } catch (const json::exception& e) {
    throw std::runtime_error(fmt::format(
        "Error parsing the glTF file '{}': {}.", gltf_path.string(), e.what()));
  }
  AddFilesFromUris(gltf_path, gltf, "buffers", &supporting_files);
  AddFilesFromUris(gltf_path, gltf, "images", &supporting_files);

  return InMemoryMesh{std::move(gltf_file), std::move(supporting_files)};
}

}  // namespace geometry
}  // namespace drake
