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
                                       double scale) {
  if (scale <= 0.0) {
    throw std::runtime_error(fmt::format(
        "ReadVtkToVolumeMesh() requires a positive scale. Given {} for '{}'.",
        scale, mesh_source.description()));
  }
  vtkNew<vtkUnstructuredGridReader> reader;
  if (mesh_source.IsPath()) {
    reader->SetFileName(mesh_source.path().c_str());
  } else {
    DRAKE_DEMAND(mesh_source.IsInMemory());
    vtkNew<vtkCharArray> char_array;
    const MemoryFile& file = mesh_source.mesh_data().mesh_file();
    const std::string& data = file.contents();
    // vtkCharArray requires a non-const pointer. Yuck. But this allows us to
    // avoid copying the string.
    char* contents_array = const_cast<char*>(data.c_str());
    char_array->SetArray(contents_array, data.size(), 1 /* don't delete */);
    reader->SetInputArray(char_array);
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
    vertices.push_back(scale * Vector3d(xyz));
  }

  std::vector<VolumeElement> elements;
  elements.reserve(vtk_mesh->GetNumberOfCells());
  auto iter =
      vtkSmartPointer<vtkCellIterator>::Take(vtk_mesh->NewCellIterator());
  for (iter->InitTraversal(); !iter->IsDoneWithTraversal();
       iter->GoToNextCell()) {
    if (iter->GetCellType() != VTK_TETRA) {
      vtkNew<vtkGenericCell> bad_cell;
      iter->GetCell(bad_cell);
      auto msg = fmt::format(
          "ReadVtkToVolumeMesh(): mesh data contains a non-tetrahedron(type "
          "id={}) cell with type id {}, dimension {}, and number of points {}: "
          "'{}'.",
          scale, static_cast<int>(VTK_TETRA), bad_cell->GetCellType(),
          bad_cell->GetCellDimension(), bad_cell->GetNumberOfPoints(),
          mesh_source.description());
      throw std::runtime_error(msg);
    }
    vtkIdList* vtk_vertex_ids = iter->GetPointIds();
    // clang-format off
    elements.emplace_back(vtk_vertex_ids->GetId(0),
                          vtk_vertex_ids->GetId(1),
                          vtk_vertex_ids->GetId(2),
                          vtk_vertex_ids->GetId(3));
    // clang-format on
  }

  return {std::move(elements), std::move(vertices)};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
