#include "drake/geometry/in_memory_mesh.h"

#include <utility>

#include <fmt/format.h>

#include "drake/common/overloaded.h"

namespace drake {
namespace geometry {

InMemoryMesh::InMemoryMesh() = default;

InMemoryMesh::InMemoryMesh(MemoryFile mesh_file,
                           string_map<FileSource> supporting_files)
    : mesh_file_(std::move(mesh_file)),
      supporting_files_(std::move(supporting_files)) {}

void InMemoryMesh::AddSupportingFile(std::string_view name,
                                     FileSource file_source) {
  if (supporting_files_.contains(name)) {
    const std::string description = std::visit(overloaded{
      [](const std::filesystem::path& path) {return path.string(); },
      [](const MemoryFile& file) { return file.filename_hint();}
    }, *this->supporting_file(name));
    throw std::runtime_error(
        fmt::format("InMemoryMesh cannot add supporting file '{}', that name "
                    "has already been used for file '{}'.",
                    name, description));
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

InMemoryMesh::~InMemoryMesh() = default;

bool InMemoryMesh::empty() const {
  return mesh_file_.contents().empty();
}

}  // namespace geometry
}  // namespace drake
