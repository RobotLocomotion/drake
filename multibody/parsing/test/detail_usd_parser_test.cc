#include "drake/multibody/parsing/detail_usd_parser.h"

#include <gmock/gmock.h>
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
    const std::string source_filename =
        filename.is_relative()
            ? FindResourceOrThrow("drake/multibody/parsing/test/" +
                                  filename.string())
            : filename.string();
    const DataSource source{DataSource::kFilename, &source_filename};
    const std::optional<std::string> parent_model_name;
    internal::CollisionFilterGroupResolver resolver{&plant_, &group_output_};
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
  CollisionFilterGroups group_output_;
};

TEST_F(UsdParserTest, NoSuchFile) {
  ParseFile("/no/such/file");
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(".*Failed to open.*"));
}

TEST_F(UsdParserTest, BoxPlane) {
  EXPECT_NO_THROW(ParseFile("usd_parser_test/box_plane.usda"));
  // TODO(rpoyner-tri) Add real test logic.
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
