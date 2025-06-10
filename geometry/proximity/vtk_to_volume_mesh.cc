#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <utility>
#include <vector>

#include <fmt/format.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkCellIterator.h>            // vtkCommonDataModel
#include <vtkCharArray.h>               // vtkCommonCore
#include <vtkNew.h>                     // vtkCommonCore
#include <vtkUnstructuredGrid.h>        // vtkCommonDataModel
#include <vtkUnstructuredGridReader.h>  // vtkIOLegacy

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

using Eigen::Vector3d;

VolumeMesh<double> ReadVtkToVolumeMesh(const MeshSource& mesh_source,
                                       const Eigen::Vector3d& scale) {
  vtkNew<vtkUnstructuredGridReader> reader;
  if (mesh_source.is_path()) {
    reader->SetFileName(mesh_source.path().c_str());
  } else {
    DRAKE_DEMAND(mesh_source.is_in_memory());
    const MemoryFile& file = mesh_source.in_memory().mesh_file;
    // Note: The contents will be copied by VTK.
    reader->SetInputString(file.contents().c_str(), file.contents().size());
    reader->SetReadFromInputString(true);
  }
  reader->Update();

  vtkUnstructuredGrid* vtk_mesh = reader->GetOutput();

  const vtkIdType num_vertices = vtk_mesh->GetNumberOfPoints();
  std::vector<Vector3<double>> vertices;
  vertices.reserve(num_vertices);
  vtkPoints* vtk_vertices = vtk_mesh->GetPoints();
  for (vtkIdType id = 0; id < num_vertices; id++) {
    double xyz[3];
    vtk_vertices->GetPoint(id, xyz);
    vertices.push_back(scale.cwiseProduct(Vector3d(xyz)));
  }

  // Mapping from vtk-file tet vertex indices to Drake tet vertex indices.
  // Depending on scale, we may have to reverse the winding.
  const bool reverse_winding = ((scale.array() < 0).cast<int>().sum() % 2) == 1;
  std::array<int, 4> v_local{0, 1, 2, 3};
  if (reverse_winding) std::swap(v_local[0], v_local[2]);

  std::vector<VolumeElement> elements;
  elements.reserve(vtk_mesh->GetNumberOfCells());
  auto iter =
      vtkSmartPointer<vtkCellIterator>::Take(vtk_mesh->NewCellIterator());
  for (iter->InitTraversal(); !iter->IsDoneWithTraversal();
       iter->GoToNextCell()) {
    if (iter->GetCellType() != VTK_TETRA) {
      vtkNew<vtkGenericCell> bad_cell;
      iter->GetCell(bad_cell);
      throw std::runtime_error(fmt::format(
          "ReadVtkToVolumeMesh(): mesh data should only contain tetrahedra "
          "(type id={}). Read cell with type id={}, dimension {}, and number "
          "of points {} in '{}'.",
          static_cast<int>(VTK_TETRA), bad_cell->GetCellType(),
          bad_cell->GetCellDimension(), bad_cell->GetNumberOfPoints(),
          mesh_source.description()));
    }
    vtkIdList* vtk_vertex_ids = iter->GetPointIds();
    // clang-format off
    elements.emplace_back(vtk_vertex_ids->GetId(v_local[0]),
                          vtk_vertex_ids->GetId(v_local[1]),
                          vtk_vertex_ids->GetId(v_local[2]),
                          vtk_vertex_ids->GetId(v_local[3]));
    // clang-format on
  }

  return {std::move(elements), std::move(vertices)};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
