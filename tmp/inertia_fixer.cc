#include <regex>

#include <drake_vendor/tinyxml2.h>
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"

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

  XMLDocument xml_doc;
  xml_doc.LoadFile(argv[1]);
  if (xml_doc.ErrorID()) {
    drake::log()->error("Failed to parse XML file: {}",
                        xml_doc.ErrorName());
    ::exit(EXIT_FAILURE);
  }

  // TODO(rpoyner-tri): Somehow, in here, inertia-fixing magic happens.

  // * Figure out which inertias need fixing, and locate their nodes in the doc.
  //   * It might be feasible to use parser internals for:
  //     * checking the model file type
  //     * hijacking the diagnostic output
  //   * URDF warning messages point to the <inertial> tag.
  //   * SDFormat warning messages point to the <link> tag.

  MultibodyPlant<double> plant{0.0};
  drake::geometry::SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);
  Parser parser{&plant};
  parser.package_map().PopulateFromRosPackagePath();

  std::vector<ModelInstanceIndex> models;
  models = parser.AddModels(argv[1]);

  // * Leverage the maybe-damaged model that got parsed by the parser:
  //   * for geometries
  //   * for mass/density parameters
  // * Build proper inertias from geometries:
  //   * cf. mujoco parser's techniques
  //   * figure out how to combine inertias for multiple geometries?
  // * Edit fixed-up inertias back into doc.

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
