#include "drake/systems/sensors/vtk_util.h"

#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolygon.h>
#include <vtkUnsignedCharArray.h>

namespace drake {
namespace systems {
namespace sensors {

vtkSmartPointer<vtkPolyData> VtkUtil::CreateSquarePlane(
    double size, const unsigned char color[3]) {
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(-size, -size, 0.);
  points->InsertNextPoint(-size,  size, 0.);
  points->InsertNextPoint( size,  size, 0.);
  points->InsertNextPoint( size, -size, 0.);

  vtkNew<vtkPolygon> polygon;
  polygon->GetPointIds()->SetNumberOfIds(4);  // Make a quad
  polygon->GetPointIds()->SetId(0, 0);
  polygon->GetPointIds()->SetId(1, 1);
  polygon->GetPointIds()->SetId(2, 2);
  polygon->GetPointIds()->SetId(3, 3);

  vtkNew<vtkCellArray> polygons;
  polygons->InsertNextCell(polygon.GetPointer());

  vtkNew<vtkUnsignedCharArray> colors;
  colors->SetNumberOfComponents(3);
  colors->SetName("Color");
  colors->InsertNextTupleValue(color);
  colors->InsertNextTupleValue(color);
  colors->InsertNextTupleValue(color);
  colors->InsertNextTupleValue(color);

  vtkNew<vtkPolyData> poly_data;
  poly_data->SetPoints(points.GetPointer());
  poly_data->SetPolys(polygons.GetPointer());
  poly_data->GetPointData()->SetScalars(colors.GetPointer());

  return vtkSmartPointer<vtkPolyData>(poly_data.GetPointer());
}

}  // namespace sensors
}  // namespace systems
}  // namespace drake
