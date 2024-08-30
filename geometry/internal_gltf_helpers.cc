#include "drake/geometry/internal_gltf_helpers.h"

#include <string>
#include <utility>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "drake/common/file_source.h"
#include "drake/common/string_map.h"

namespace drake {
namespace geometry {
namespace internal {

using nlohmann::json;

namespace {

void AddFilesFromUris(const std::filesystem::path gltf_path, const json& gltf,
                      std::string_view array_name,
                      string_map<FileSource>* supporting_files) {
  auto& array = gltf[array_name];
  for (size_t i = 0; i < array.size(); ++i) {
    auto& item = array[i];
    if (item.contains("uri") && item["uri"].is_string()) {
      const std::string_view uri = item["uri"].template get<std::string_view>();
      if (uri.find("data:") != std::string::npos) {
        continue;
      }
      supporting_files->emplace(uri, gltf_path.parent_path() / uri);
    }
  }
}

}  // namespace

InMemoryMesh PreParseGltf(const std::filesystem::path gltf_path,
                          bool include_images) {
  auto gltf_file = MemoryFile::Make(gltf_path);

  string_map<FileSource> supporting_files;
  json gltf;
  try {
    gltf = json::parse(gltf_file.contents());
  } catch (const json::exception& e) {
    throw std::runtime_error(
        fmt::format("Couldn't compute convex hull for glTF file '{}', there "
                    "is an error in the file: {}.",
                    gltf_path.string(), e.what()));
  }
  AddFilesFromUris(gltf_path, gltf, "buffers", &supporting_files);
  if (include_images) {
    AddFilesFromUris(gltf_path, gltf, "images", &supporting_files);
  }

  return InMemoryMesh(std::move(gltf_file), std::move(supporting_files));
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
