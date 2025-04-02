#include "drake/geometry/mesh_source.h"

#include <algorithm>

namespace drake {
namespace geometry {
namespace {

std::string GetExtensionLower(const std::filesystem::path& file_path) {
  std::string ext = file_path.extension();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return ext;
}

}  // namespace

MeshSource::MeshSource(std::filesystem::path path)
    : source_(std::move(path)), extension_(GetExtensionLower(this->path())) {}

MeshSource::MeshSource(InMemoryMesh&& mesh)
    : source_(std::move(mesh)), extension_(in_memory().mesh_file.extension()) {}

MeshSource::~MeshSource() = default;

std::string MeshSource::description() const {
  return is_path()
             ? (path().empty() ? std::string("<empty path>") : path().string())
             : (in_memory().mesh_file.filename_hint().empty()
                    ? std::string("<no filename hint given>")
                    : in_memory().mesh_file.filename_hint());
}

}  // namespace geometry
}  // namespace drake
