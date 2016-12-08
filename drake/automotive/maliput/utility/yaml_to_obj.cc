/// @file yaml_to_obj.cc
///
/// Take a yaml file as input, build the resulting monolane road geometry, and
/// render the road surface to a WaveFront OBJ output file.
#include <iostream>
#include <string>

#include <gflags/gflags.h>

#include "drake/automotive/maliput/monolane/loader.h"
#include "drake/automotive/maliput/utility/generate_obj.h"

namespace mono = drake::maliput::monolane;

DEFINE_string(yaml_file, "",
              "yaml input file defining a monolane road geometry");
DEFINE_string(obj_dir, ".", "Directory to contain rendered road surface");
DEFINE_string(obj_file, "",
              "Basename for output Wavefront OBJ and MTL files");
DEFINE_double(grid_size, 1.,
              "Size of a grid unit in the rendered mesh covering the"
              " road surface");

int main(int argc, char* argv[]) {
  std::cerr << "main() !\n";
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_yaml_file.empty()) {
    std::cerr << "no input file!\n";
    return 1;
  }
  if (FLAGS_obj_file.empty()) {
    std::cerr << "no output file!\n";
    return 1;
  }
  std::cerr << "loading road geometry !\n";
  auto rg = mono::LoadFile(FLAGS_yaml_file);


  std::cerr << "generating obj !\n";
  drake::maliput::utility::GenerateObjFile(rg.get(),
                                           FLAGS_obj_dir,
                                           FLAGS_obj_file,
                                           FLAGS_grid_size);

  return 0;
}
