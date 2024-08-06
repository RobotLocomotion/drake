#pragma once

/* @file
 A collection of utilities for working with VTK and glTF files. */

#include <filesystem>
#include <string>

#include <vtkMatrix4x4.h>             // vtkCommonMath
#include <vtkMemoryResourceStream.h>  // vtkIOCore
#include <vtkURI.h>                   // vtkIOCore
#include <vtkURILoader.h>             // vtkIOCore

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
#include "drake/common/file_contents.h"
#include "drake/common/string_map.h"
#include "drake/geometry/in_memory_mesh.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

/* This helps serve the family of in-memory glTF files (.bin, .png, etc.) to
 VTK's document loader, turning file URIs in the main glTF file into resource
 streams drawing from memory. */
class MeshMemoryLoader final : public vtkURILoader {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MeshMemoryLoader);

  /* Constructs the loader for a family of related mesh files.
   @param data  The map of related files. The key for each file will be a file
                URL referred to by other files -- typically the glTF file.
                The data will be aliased and must stay alive longer than this.
   */
  explicit MeshMemoryLoader(const string_map<common::FileContents>* data);

 private:
  vtkSmartPointer<vtkResourceStream> DoLoad(const vtkURI& uri) final;

  const std::string name_;
  const string_map<common::FileContents>& mesh_data_;
  vtkSmartPointer<vtkURI> base_uri_;
};

/* Given a file path to a .gltf file, loads the .gltf file contents into memory
 along with any supporting .bin and .image files. Note: in applications where
 only the geometry matters, images can be excluded by setting `include_images`
 to false. */
InMemoryMesh PreParseGltf(const std::filesystem::path gltf_path,
                          bool include_images = true);

/* Multiplies the position vector p_AQ by the transform T_BA, returning p_BQ. */
Eigen::Vector3d VtkMultiply(vtkMatrix4x4* T_BA, const Eigen::Vector3d& p_AQ);

/* Creates a vtk resource stream based on the provide string's contents. The
 `source` string will be aliased and must remain alive as long as the returned
 stream. */
vtkSmartPointer<vtkResourceStream> MakeStreamForString(
    const std::string* source);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
