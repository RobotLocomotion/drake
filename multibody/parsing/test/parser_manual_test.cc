#include <iostream>

#include <gflags/gflags.h>

#include "drake/common/filesystem.h"
#include "drake/multibody/parsing/parser.h"

DEFINE_bool(scene_graph, true, "include/exclude scene graph");
// TODO(rpoyner-tri): support parser flags, e.g. strict
// TODO(rpoyner-tri): support adding package.xml files

namespace drake {
namespace multibody {
namespace {

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[INPUT-FILE]\n"
                          "Run multibody parser; print errors if any");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    std::cerr << "missing input filename\n";
    return 1;
  }
  MultibodyPlant<double> plant{0.0};
  drake::geometry::SceneGraph<double> scene_graph;
  if (FLAGS_scene_graph) {
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
  }
  Parser parser{&plant};

  std::cerr << "parsing...\n";
  auto models = parser.AddAllModelsFromFile(argv[1]);
  std::cout << "Parsed " << models.size() << " models.\n";
  return models.empty();
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
