#include "drake/geometry/vtk_gltf_uri_loader.h"

#include <algorithm>
#include <utility>

#include <fmt/format.h>
#include <vtkFileResourceStream.h>    // vtkIOCore
#include <vtkMemoryResourceStream.h>  // vtkIOCore

#include "drake/common/drake_assert.h"
#include "drake/common/memory_file.h"
#include "drake/common/overloaded.h"

namespace drake {
namespace geometry {
namespace internal {
namespace {

namespace fs = std::filesystem;

constexpr std::string_view kBaseUriPrefix("/VtkGltfUriLoader/");

/* Creates a vtk file stream for the given gltf-related path.

 @param file_path    The path, resolvable from the current working directory, to
                     open.
 @param gltf_name    The name of the glTF file to display in case of error.
 @param file_label   A user-facing label for the file (referenced by file_path)
                     to display in case of error.
 @throws std::exception if the path cannot be opened.*/
vtkSmartPointer<vtkResourceStream> OpenFileStream(const fs::path& file_path,
                                                  const std::string& gltf_name,
                                                  std::string_view file_label) {
  vtkNew<vtkFileResourceStream> stream;
  if (!stream->Open(file_path.c_str())) {
    throw std::runtime_error(fmt::format(
        "glTF '{}', named an inaccessible file: '{}'.", gltf_name, file_label));
  }
  return stream;
}

/* Creates a vtk resource stream based on the provided string's contents. The
 `source` string will be aliased and must remain alive as long as the returned
 stream. */
vtkSmartPointer<vtkResourceStream> OpenStringStream(const std::string* source) {
  vtkNew<vtkMemoryResourceStream> stream;
  stream->SetBuffer(source->c_str(), source->size(), /* copy= */ false);
  return stream;
}

}  // namespace

vtkStandardNewMacro(VtkGltfUriLoader);

VtkGltfUriLoader::~VtkGltfUriLoader() = default;

void VtkGltfUriLoader::SetMeshSource(const MeshSource* source) {
  if (source == nullptr) {
    throw std::runtime_error(
        "VtkGltfUriLoader::SetMeshSource() requires a non-null source.");
  }
  source_ = source;
}

vtkSmartPointer<vtkResourceStream> VtkGltfUriLoader::MakeGltfStream() const {
  if (source_ == nullptr) {
    throw std::runtime_error(
        "VtkGltfUriLoader::MakeGltfStream() requires a non-null source. Set it "
        "with a call to SetMeshSource().");
  }
  if (source_->is_path()) {
    return OpenFileStream(source_->path(), source_->description(),
                          "<the .gltf file itself>");
  } else {
    return OpenStringStream(&source_->in_memory().mesh_file.contents());
  }
}

VtkGltfUriLoader::VtkGltfUriLoader() {
  base_uri_ = vtkURI::Make("file", vtkURIComponent(), kBaseUriPrefix.data());
  SetBaseURI(base_uri_);
}

vtkSmartPointer<vtkResourceStream> VtkGltfUriLoader::DoLoad(const vtkURI& uri) {
  if (source_ == nullptr) {
    throw std::runtime_error(
        "Exercising a VtkGltfUriLoader instance requires a non-null source. "
        "Set it with a call to SetMeshSource().");
  }

  // It's not clear if VTK enforces lower case; to make Drake not care, we'll
  // force the schema to lower case.
  std::string scheme = uri.GetScheme().GetValue();
  std::transform(scheme.begin(), scheme.end(), scheme.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });
  if (scheme == "file") {
    const std::string& uri_path = uri.GetPath().GetValue();

    // File URIs came from this uri loader.
    DRAKE_DEMAND(uri_path.substr(0, kBaseUriPrefix.size()) == kBaseUriPrefix);
    const std::string_view name(uri_path.begin() + kBaseUriPrefix.size(),
                                uri_path.end());

    if (source_->is_path()) {
      const fs::path& gltf_path = source_->path();
      const fs::path absolute_path = gltf_path.parent_path() / name;
      return OpenFileStream(absolute_path, source_->description(), name);
    } else {
      const auto file_source_iter =
          source_->in_memory().supporting_files.find(name);
      if (file_source_iter == source_->in_memory().supporting_files.end()) {
        // If the glTF file refers to a file that isn't in our set of
        // supporting files, we won't immediately throw. We'll let the parser
        // yell about it later, when it's actually needed.
        return nullptr;
      }

      return std::visit<vtkSmartPointer<vtkResourceStream>>(
          overloaded{[this, name](const fs::path& source_path) {
                       return OpenFileStream(source_path,
                                             source_->description(), name);
                     },
                     [](const MemoryFile& file) {
                       return OpenStringStream(&file.contents());
                     }},
          file_source_iter->second);
    }
  } else if (scheme == "data") {
    // For data URIs, we'll let VTK's infrastructure handle it.
    return this->LoadData(uri);
  }

  vtkErrorMacro("Unknown URI scheme for \"" << uri.ToString() << "\"");
  return nullptr;
}
}  // namespace internal
}  // namespace geometry
}  // namespace drake
