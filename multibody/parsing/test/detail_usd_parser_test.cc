#include "drake/multibody/parsing/detail_usd_parser.h"

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/diagnostic_policy_test_base.h"

namespace drake {
namespace multibody {
namespace internal {
namespace {

namespace fs = std::filesystem;

using drake::internal::DiagnosticPolicy;
using geometry::SceneGraph;

class UsdParserTest : public test::DiagnosticPolicyTestBase {
 public:
  UsdParserTest() { plant_.RegisterAsSourceForSceneGraph(&scene_graph_); }

  std::vector<ModelInstanceIndex> ParseFile(const fs::path& filename) {
    const std::string source_filename = filename.string();
    const DataSource source{DataSource::kFilename, &source_filename};
    const std::optional<std::string> parent_model_name;
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       &plant_,  &resolver,    NoSelect};
    UsdParser dut;
    auto result = dut.AddAllModels(source, parent_model_name, w);
    resolver.Resolve(diagnostic_policy_);
    return result;
  }

  // USD cannot delegate to any other parsers.
  static ParserInterface& NoSelect(const drake::internal::DiagnosticPolicy&,
                                   const std::string&) {
    DRAKE_UNREACHABLE();
  }

 protected:
  ParsingOptions options_;
  PackageMap package_map_;
  MultibodyPlant<double> plant_{0.01};
  SceneGraph<double> scene_graph_;
};

// Finds a file resource within 'usd_parser_test'.
std::string FindUsdTestResourceOrThrow(const std::string& filename) {
    const std::string resource_dir{
      "drake/multibody/parsing/test/usd_parser_test/"};
    return FindResourceOrThrow(resource_dir + filename);
}

TEST_F(UsdParserTest, BasicImportTest) {
  // hong-nvidia: temporary check to ensure that this file indeed exists
  drake::log()->info(FindUsdTestResourceOrThrow("cube_plane.usda"));

  const fs::path filename{
    "multibody/parsing/test/usd_parser_test/cube_plane.usda"};
  ParseFile(filename);
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
