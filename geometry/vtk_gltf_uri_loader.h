#pragma once

#include <filesystem>
#include <string>

#include <vtkURI.h>        // vtkIOCore
#include <vtkURILoader.h>  // vtkIOCore

#include "drake/common/drake_copyable.h"
#include "drake/common/drake_export.h"
#include "drake/geometry/mesh_source.h"

namespace drake {
namespace geometry {
namespace internal DRAKE_NO_EXPORT {

/* This serves the family of glTF files (.gltf, .bin, .png, etc.) to VTK's
 document loader, while masking the details of the mesh source. Based on the
 mesh source semantics, generates appropriately typed VTK resource streams. */
class VtkGltfUriLoader final : public vtkURILoader {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VtkGltfUriLoader);

  /* The VTK boilerplate to make a successfully reference-counted object. */
  static VtkGltfUriLoader* New();

  /* Sets the source for this URI loader. It must be called before any other
   API.
   @throws if `source` is null.*/
  void SetMeshSource(const MeshSource* source);

  /* Creates a vtk resource stream for the main .gltf file.
   @pre the source has been set with a call to SetMeshSource(). */
  vtkSmartPointer<vtkResourceStream> MakeGltfStream() const;

  ~VtkGltfUriLoader() final;

 private:
  /* Constructs the loader without a source. Necessary for VTK's reference
   counting infrastructure. */
  VtkGltfUriLoader();

  vtkSmartPointer<vtkResourceStream> DoLoad(const vtkURI& uri) final;

  const std::string name_;
  const MeshSource* source_{};
  vtkSmartPointer<vtkURI> base_uri_;
};

}  // namespace internal
}  // namespace geometry
}  // namespace drake
