#pragma once

/* @file
 A collection of utilities for working with VTK and glTF files. */

#include <filesystem>
#include <string>

#include <vtkURI.h>        // vtkIOCore
#include <vtkURILoader.h>  // vtkIOCore

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_export.h"
#include "drake/common/eigen_types.h"
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
  explicit MeshMemoryLoader(const InMemoryMesh* data);

 private:
  vtkSmartPointer<vtkResourceStream> DoLoad(const vtkURI& uri) final;

  const std::string name_;
  const InMemoryMesh& mesh_;
  vtkSmartPointer<vtkURI> base_uri_;
};

/* Creates a vtk resource stream based on the provided string's contents. The
 `source` string will be aliased and must remain alive as long as the returned
 stream. */
vtkSmartPointer<vtkResourceStream> MakeStreamForString(
    const std::string* source);

}  // namespace internal
}  // namespace geometry
}  // namespace drake
