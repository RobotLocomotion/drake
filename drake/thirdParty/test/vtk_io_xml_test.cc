#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>

// Read a point cloud.
// Derived from VTK examples, VTK/Examples/Cxx/...:
//  * IO/ReadPolyData
//  * IO/WriteVTP
//  * PolyData/PointSource

int main() {
  // Read a file that was generated with a random point cloud.
  const char* filename = "random.vtp";
  vtkSmartPointer<vtkXMLPolyDataReader> reader =
    vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(filename);
  reader->Update();

  return 0;
}
