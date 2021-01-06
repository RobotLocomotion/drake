#include <gflags/gflags.h>
#include <vtkOBJWriter.h>
#include <vtkSTLReader.h>

#include "drake/common/drake_throw.h"

DEFINE_string(input, "", "STL filename to read");
DEFINE_string(output, "", "OBJ filename to write");

namespace drake {
namespace {

void main() {
  DRAKE_THROW_UNLESS(!FLAGS_input.empty());
  DRAKE_THROW_UNLESS(!FLAGS_output.empty());

  auto reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(FLAGS_input.c_str());
  reader->Update();

  auto writer = vtkSmartPointer<vtkOBJWriter>::New();
  writer->SetFileName(FLAGS_output.c_str());
  writer->SetInputData(reader->GetOutput());
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
