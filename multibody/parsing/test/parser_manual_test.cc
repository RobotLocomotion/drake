#include <gflags/gflags.h>

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
    drake::log()->error("missing input filename");
    return 1;
  }
  MultibodyPlant<double> plant{0.0};
  drake::geometry::SceneGraph<double> scene_graph;
  if (FLAGS_scene_graph) {
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
  }
  Parser parser{&plant};

  drake::log()->info("parsing {}", argv[1]);
  std::vector<ModelInstanceIndex> models = parser.AddModelsFromFile(argv[1]);
  drake::log()->info("Parsed {} models.", models.size());
  return models.empty();
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
