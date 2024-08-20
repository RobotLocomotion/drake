#include "drake/geometry/in_memory_mesh.h"

#include <algorithm>
#include <utility>

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

MeshSource::MeshSource(std::filesystem::path path) : source_(std::move(path)) {
  extension_ = GetExtensionLower(this->path());
}

MeshSource::MeshSource(InMemoryMesh mesh, std::string extension)
    : source_(std::move(mesh)), extension_(std::move(extension)) {}

std::string MeshSource::description() const {
  return IsPath() ? path().string()
                  : mesh_data().mesh_file.filename_hint();
}

}  // namespace geometry
}  // namespace drake
