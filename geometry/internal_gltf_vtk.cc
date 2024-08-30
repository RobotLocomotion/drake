#include "drake/geometry/internal_gltf_vtk.h"

#include <algorithm>
#include <utility>

#include <fmt/format.h>
#include <vtkFileResourceStream.h>    // vtkIOCore
#include <vtkMemoryResourceStream.h>  // vtkIOCore

#include "drake/common/drake_assert.h"
#include "drake/common/memory_file.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

namespace {

constexpr char kBaseUriPrefix[] = "/MeshMemoryLoader/";
constexpr int kBaseUriPrefxLength = 18;
}

MeshMemoryLoader::MeshMemoryLoader(const InMemoryMesh* mesh)
    : mesh_(*mesh) {
  DRAKE_DEMAND(mesh != nullptr);
  base_uri_ = vtkURI::Make("file", vtkURIComponent(), kBaseUriPrefix);
  SetBaseURI(base_uri_);
}

vtkSmartPointer<vtkResourceStream> MeshMemoryLoader::DoLoad(const vtkURI& uri) {
  // Case insensitive
  std::string scheme = uri.GetScheme().GetValue();
  std::transform(scheme.begin(), scheme.end(), scheme.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });
  if (scheme == "file") {
    // Populate the stream with the data.
    const std::string& path = uri.GetPath().GetValue();
    // Note: I the glTF file were to reference a file in a *higher* directory
    // (e.g., ../foo.bin), then this trick will not work. But the glTF spec
    // explicitly disallows that.
    auto pos = path.find(kBaseUriPrefix);
    if (pos == std::string::npos) {
      throw std::runtime_error(fmt::format(
          "Error with in-memory glTF file. The glTF file has an unrecognizable "
          "resource file URI: '{}'. All URIs should use relative file "
          "positions and be included in the in-memory mesh's supporting files.",
          uri.ToString()));
    }
    const std::string name = path.substr(pos + kBaseUriPrefxLength);
    const FileSource* file_source = mesh_.supporting_file(name);
    if (file_source == nullptr || file_source->empty()) {
      // If the glTF file refers to a file that isn't in our set of
      // supporting files, we won't immediately throw. We'll let the parser yell
      // about it later, when it's actually needed.
      return nullptr;
    }
    if (file_source->is_path()) {
      vtkNew<vtkFileResourceStream> stream;
      stream->Open(file_source->path().c_str());
      return stream;
    } else {
      DRAKE_DEMAND(file_source->is_memory_file());
      const MemoryFile& file = file_source->memory_file();
      vtkNew<vtkMemoryResourceStream> stream;
      stream->SetBuffer(file.contents().c_str(), file.contents().size(),
                        /* copy= */ false);
      return stream;
    }
  } else if (scheme == "data") {
    // For data URIs, we'll let VTK's infrastructure handle it.
    return this->LoadData(uri);
  }

  vtkErrorMacro("Unknown URI scheme for \"" << uri.ToString() << "\"");
  return nullptr;
}

vtkSmartPointer<vtkResourceStream> MakeStreamForString(
    const std::string* source) {
  vtkNew<vtkMemoryResourceStream> stream;
  stream->SetBuffer(source->c_str(), source->size(), /* copy= */ false);
  return stream;
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
