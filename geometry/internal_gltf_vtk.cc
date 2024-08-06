#include "drake/geometry/internal_gltf_vtk.h"

#include <string>

#include <fmt/format.h>
#include <nlohmann/json.hpp>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {
namespace internal {

using common::FileContents;
using Eigen::Vector3d;
using nlohmann::json;

MeshMemoryLoader::MeshMemoryLoader(const string_map<FileContents>* data)
    : mesh_data_(*data) {
  DRAKE_DEMAND(data != nullptr);
  base_uri_ = vtkURI::Make("file", vtkURIComponent(), "/convex_hull/");
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
    // TODO(SeanCurtis-TRI): IF the glTF file references a file in a *higher*
    // directory (e.g., ../common.bin), then this trick will not work.
    auto pos = path.find("/convex_hull/");
    if (pos == std::string::npos) {
      throw std::runtime_error(fmt::format(
          "Can't compute convex hull for in-memory glTF file. The glTF file "
          "has an unrecognizable resource URI: '{}'. All URIs should be "
          "use relative file positions and be included in the in-memory "
          "mesh's supporting files.",
          uri.ToString()));
    }
    const std::string name = path.substr(pos + 13);
    if (!mesh_data_.contains(name)) {
      // If the glTF file refers to a file that isn't in our set of
      // supporting files, we won't immediately throw. We'll wait to see if
      // it's an actual parsing problem.
      return nullptr;
    }
    const FileContents& file_data = mesh_data_.at(name);
    vtkNew<vtkMemoryResourceStream> stream;
    stream->SetBuffer(file_data.contents().c_str(), file_data.contents().size(),
                      /* copy= */ false);
    return stream;
  } else if (scheme == "data") {
    // For data URIs, we'll let VTK's infrastructure handle it.
    return this->LoadData(uri);
  }

  vtkErrorMacro("Unknown URI scheme for \"" << uri.ToString() << "\"");
  return nullptr;
}

namespace {

void AddFilesFromUris(const std::filesystem::path gltf_path, const json& gltf,
                      std::string_view array_name,
                      string_map<FileContents>* supporting_files) {
  auto& array = gltf[array_name];
  for (size_t i = 0; i < array.size(); ++i) {
    auto& item = array[i];
    if (item.contains("uri") && item["uri"].is_string()) {
      const std::string_view uri =
          item["uri"].template get<std::string_view>();
      if (uri.find("data:") != std::string::npos) {
        continue;
      }
      supporting_files->emplace(
          uri, FileContents::Make(gltf_path.parent_path() / uri));
    }
  }
}

}  // namespace

InMemoryMesh PreParseGltf(const std::filesystem::path gltf_path,
                          bool include_images) {
  auto gltf_file = FileContents::Make(gltf_path);

  string_map<FileContents> supporting_files;
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

  return {std::move(gltf_file), std::move(supporting_files)};
}

Vector3d VtkMultiply(vtkMatrix4x4* T_BA, const Vector3d& p_AQ) {
  double p_in[] = {p_AQ.x(), p_AQ.y(), p_AQ.z(), 1};
  double p_out[4];
  T_BA->MultiplyPoint(p_in, p_out);
  return Vector3d(p_out);
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
