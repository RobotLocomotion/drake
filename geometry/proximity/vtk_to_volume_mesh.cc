#include "drake/geometry/proximity/vtk_to_volume_mesh.h"

#include <utility>
#include <vector>

#include <vtkCellIterator.h>
#include <vtkNew.h>
#include <vtkUnstructuredGrid.h>
#include <vtkUnstructuredGridReader.h>

#include "drake/common/eigen_types.h"

namespace drake {
namespace geometry {
namespace internal {

VolumeMesh<double> ReadVtkToVolumeMesh(const std::string& filename) {
  vtkNew<vtkUnstructuredGridReader> reader;
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkUnstructuredGrid* vtk_mesh = reader->GetOutput();

  std::vector<Vector3<double>> vertices;
  vtkIdType num_vertices = vtk_mesh->GetNumberOfPoints();
  vtkPoints* vtk_vertices = vtk_mesh->GetPoints();
  for (vtkIdType id = 0; id < num_vertices; id++) {
    double xyz[3];
    vtk_vertices->GetPoint(id, xyz);
    vertices.emplace_back(xyz);
  }

  std::vector<VolumeElement> elements;
  vtkCellIterator* iter = vtk_mesh->NewCellIterator();
  for (iter->InitTraversal(); !iter->IsDoneWithTraversal();
       iter->GoToNextCell()) {
    vtkIdList* vtk_vertex_ids = iter->GetPointIds();
    elements.emplace_back(vtk_vertex_ids->GetId(0),
                          vtk_vertex_ids->GetId(1),
                          vtk_vertex_ids->GetId(2),
                          vtk_vertex_ids->GetId(3));
  }
  iter->Delete();

  return {std::move(elements), std::move(vertices)};
}

}  // namespace internal
}  // namespace geometry
}  // namespace drake
