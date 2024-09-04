#include "drake/geometry/in_memory_mesh.h"

#include <algorithm>

#include <fmt/format.h>

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

InMemoryMesh::InMemoryMesh() = default;

InMemoryMesh::InMemoryMesh(MemoryFile mesh_file,
                           string_map<FileSource> supporting_files)
    : mesh_file_(std::move(mesh_file)),
      supporting_files_(std::move(supporting_files)) {}

void InMemoryMesh::AddSupportingFile(std::string_view name,
                                     FileSource file_source) {
  if (supporting_files_.contains(name)) {
    throw std::runtime_error(
        fmt::format("InMemoryMesh cannot add supporting file '{}', that name "
                    "has already been used for file '{}'.",
                    name, this->supporting_file(name)->description()));
  }
  supporting_files_.emplace(name, std::move(file_source));
}

std::vector<std::string_view> InMemoryMesh::SupportingFileNames() const {
  std::vector<std::string_view> names;
  for (const auto& [name, _] : supporting_files_) {
    names.emplace_back(name);
  }
  return names;
}

const FileSource* InMemoryMesh::supporting_file(std::string_view name) const {
  const auto iter = supporting_files_.find(name);
  if (iter == supporting_files_.end()) {
    return nullptr;
  }
  return &iter->second;
}

bool InMemoryMesh::empty() const {
  return mesh_file_.contents().empty();
}

MeshSource::MeshSource(std::filesystem::path path) : source_(std::move(path)) {
  extension_ = GetExtensionLower(this->path());
}

MeshSource::MeshSource(InMemoryMesh mesh) : source_(std::move(mesh)) {
  extension_ = in_memory().mesh_file().extension();
}

std::string MeshSource::description() const {
  return is_path() ? path().string() : in_memory().mesh_file().filename_hint();
}

}  // namespace geometry
}  // namespace drake
