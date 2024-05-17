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

  std::vector<ModelInstanceIndex> ParseFile(const std::string& filename) {
    const DataSource source{DataSource::kFilename, &filename};
    const std::optional<std::string> parent_model_name;
    internal::CollisionFilterGroupResolver resolver{&plant_, &group_output_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       &plant_,  &resolver,    NoSelect};
    UsdParserWrapper dut;
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

// Finds a file resource within 'usd_parser_test'.
std::string FindUsdTestResourceOrThrow(const std::string& filename) {
    const std::string resource_dir{
      "drake/multibody/parsing/test/usd_parser_test/"};
    return FindResourceOrThrow(resource_dir + filename);
}

TEST_F(UsdParserTest, NoSuchFile) {
  ParseFile("/no/such/file");
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(".*File does not exist.*"));
}

TEST_F(UsdParserTest, BasicImportTest) {
  std::string filename = FindUsdTestResourceOrThrow("simple_geometries.usda");
  ParseFile(filename);
  EXPECT_EQ(plant_.num_bodies(), 5);
  EXPECT_EQ(plant_.num_collision_geometries(), 11);
  EXPECT_EQ(plant_.num_visual_geometries(), 11);
}

TEST_F(UsdParserTest, MissingMetadataTest) {
  std::string filename =
    FindUsdTestResourceOrThrow("invalid/missing_metadata.usda");
  ParseFile(filename);
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
    ".*Failed to read metersPerUnit in stage metadata.*"));
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
    ".*Failed to read upAxis in stage metadata.*"));
}

TEST_F(UsdParserTest, InvalidMassTest) {
  std::string filename =
    FindUsdTestResourceOrThrow("invalid/invalid_mass.usda");
  ParseFile(filename);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
    ".*Double precision float is not supported by UsdPhysicsMassAPI.*"));
  EXPECT_THAT(TakeWarning(), ::testing::MatchesRegex(
    ".*Failed to read the mass of the prim at.*"));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
