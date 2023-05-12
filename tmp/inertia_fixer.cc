#include <regex>

#include <drake_vendor/tinyxml2.h>
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"

DEFINE_bool(scene_graph, true, "include/exclude scene graph");

namespace drake {
namespace multibody {
namespace {

using tinyxml2::XMLNode;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[INPUT-FILE-OR-URL]\n"
                          "Rewrite URDF/SDFormat with fixed-up inertias");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }
  const char* outfile = (argc > 2) ? argv[2] : "/dev/stdout";

  // Hardcode a log pattern that gives us un-decorated error messages.
  // This defeats the `-spdlog_pattern` command line option; oh well.
  drake::logging::set_log_pattern("%v");

  MultibodyPlant<double> plant{0.0};
  drake::geometry::SceneGraph<double> scene_graph;
  if (FLAGS_scene_graph) {
    plant.RegisterAsSourceForSceneGraph(&scene_graph);
  }
  Parser parser{&plant};
  parser.package_map().PopulateFromRosPackagePath();

  const bool is_url = std::regex_search(
      argv[1], std::regex("^[A-Za-z0-9+.-]+://"));

  drake::log()->info("parsing {}", argv[1]);

  // Chances are good that parsing may end in an error (implemented as throw).
  // For purposes of this tool, we should make its message usable. To do that,
  // catch it and display it via logging. Yes, this violates the coding
  // standard; in this case it makes the tool significantly more useful.
  std::vector<ModelInstanceIndex> models;
  try {
    if (is_url) {
      models = parser.AddModelsFromUrl(argv[1]);
    } else {
      models = parser.AddModels(argv[1]);
    }
  } catch (const std::exception& e) {
    drake::log()->error(e.what());
    ::exit(EXIT_FAILURE);
  }

  drake::log()->info("Parsed {} models.", models.size());


  drake::log()->info("\n\n\nXML Text\n\n\n");

  XMLDocument xml_doc;
  xml_doc.LoadFile(argv[1]);

  // TODO(rpoyner-tri): Somehow, in here, inertia-fixing magic happens.

  // tinyxml2 preserves the input text almost exactly. Probably good enough for
  // this purpose.
  xml_doc.SaveFile(outfile);


  return models.empty();
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}
