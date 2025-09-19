/* @file This contains the mesh refinement functions for pydrake.geometry. */

#include "drake/bindings/generated_docstrings/geometry_proximity.h"
#include "drake/bindings/pydrake/geometry/geometry_py.h"
#include "drake/geometry/mesh_source.h"
#include "drake/geometry/proximity/volume_mesh.h"
#include "drake/geometry/proximity/volume_mesh_refiner.h"

namespace drake {
namespace pydrake {

void DefineGeometryRefine(py::module m) {
  // NOLINTNEXTLINE(build/namespaces): Emulate placement in namespace.
  using namespace drake::geometry;
  constexpr auto& doc = pydrake_doc_geometry_proximity.drake.geometry;

  m.def("RefineVolumeMesh", &RefineVolumeMesh, py::arg("mesh"),
      doc.RefineVolumeMesh.doc);

  m.def("RefineVolumeMeshIntoVtkFileContents",
      &RefineVolumeMeshIntoVtkFileContents, py::arg("mesh_source"),
      doc.RefineVolumeMeshIntoVtkFileContents.doc);
}

}  // namespace pydrake
}  // namespace drake
