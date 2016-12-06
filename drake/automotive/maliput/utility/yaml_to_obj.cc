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
DEFINE_string(obj_file, "yaml_to_obj.obj",
              "WaveFront OBJ output file"
              " to contain the rendered road surface");

int main(int argc, char* argv[]) {
  std::cerr << "main() !\n";
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_yaml_file.empty()) {
    std::cerr << "no input file!\n";
    return 1;
  }
  std::cerr << "loading road geometry !\n";
  auto rg = mono::LoadFile(FLAGS_yaml_file);


  std::cerr << "generating obj !\n";
  drake::maliput::utility::generate_obj(rg.get(), FLAGS_obj_file, 1.);

  return 0;
}
