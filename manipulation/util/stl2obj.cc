#include <gflags/gflags.h>

// To ease build system upkeep, we annotate VTK includes with their deps.
#include <vtkDecimatePro.h>     // vtkFiltersCore
#include <vtkOBJWriter.h>       // vtkIOGeometry
#include <vtkSTLReader.h>       // vtkIOGeometry
#include <vtkTriangleFilter.h>  // vtkFiltersCore

#include "drake/common/drake_assert.h"
#include "drake/common/text_logging.h"

DEFINE_string(input, "", "STL filename to read");
DEFINE_string(output, "", "OBJ filename to write");
DEFINE_double(reduction, 0.0, "Remove this fraction of the vertices (< 1.0)");

namespace drake {
namespace {

void main() {
  DRAKE_THROW_UNLESS(!FLAGS_input.empty());
  DRAKE_THROW_UNLESS(!FLAGS_output.empty());
  DRAKE_THROW_UNLESS(FLAGS_reduction >= 0.0);
  DRAKE_THROW_UNLESS(FLAGS_reduction < 1.0);

  // Load the STL.
  auto reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(FLAGS_input.c_str());
  reader->Update();
  vtkPolyData* to_be_written = reader->GetOutput();
  log()->debug("The STL mesh has {} points and {} polygons",
               to_be_written->GetNumberOfPoints(),
               to_be_written->GetNumberOfPolys());

  // Optionally decimate.  We must to create these objects outside of the
  // "if" block (not as temporaries inside) so that they remain available
  // for the OBJWriter to read from, below.
  auto triangle_reader = vtkSmartPointer<vtkTriangleFilter>::New();
  auto decimate = vtkSmartPointer<vtkDecimatePro>::New();
  if (FLAGS_reduction > 0.0) {
    triangle_reader->SetInputData(to_be_written);
    triangle_reader->Update();

    decimate->SetInputData(triangle_reader->GetOutput());
    decimate->PreserveTopologyOff();
    decimate->SplittingOn();
    decimate->BoundaryVertexDeletionOn();
    decimate->SetMaximumError(VTK_DOUBLE_MAX);
    decimate->SetTargetReduction(FLAGS_reduction);
    decimate->Update();

    to_be_written = decimate->GetOutput();
  }

  // Write the OBJ.
  log()->debug("The OBJ mesh has {} points and {} polygons",
               to_be_written->GetNumberOfPoints(),
               to_be_written->GetNumberOfPolys());
  auto writer = vtkSmartPointer<vtkOBJWriter>::New();
  writer->SetFileName(FLAGS_output.c_str());
  writer->SetInputData(to_be_written);
  writer->Write();
}

}  // namespace
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      R"""(Reads in an *.stl mesh and writes it out as an *.obj mesh.

NOTE: The mesh conversion simply uses VTK's default mesh reading and writing.
As with any automated conversion tool, the output might not satisfy your
requirements. Be careful to confirm that the conversion meets your needs.
)""");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::main();
  return 0;
}
