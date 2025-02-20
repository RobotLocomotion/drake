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

  std::vector<ModelInstanceIndex> ParseFile(const DataSource& source) {
    const std::optional<std::string> parent_model_name;
    internal::CollisionFilterGroupResolver resolver{&plant_};
    ParsingWorkspace w{options_, package_map_, diagnostic_policy_,
                       nullptr,  &plant_,      &resolver,
                       NoSelect};
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
};

// Finds a file resource within 'usd_parser_test'.
std::string FindUsdTestResourceOrThrow(const std::string& filename) {
  const std::string resource_dir{
      "drake/multibody/parsing/test/usd_parser_test/"};
  return FindResourceOrThrow(resource_dir + filename);
}

TEST_F(UsdParserTest, BasicImportTest) {
  std::string filename = FindUsdTestResourceOrThrow("simple_geometries.usda");
  const DataSource source{DataSource::kFilename, &filename};
  ParseFile(source);
  EXPECT_EQ(plant_.num_bodies(), 5);
  EXPECT_EQ(plant_.num_collision_geometries(), 11);
  EXPECT_EQ(plant_.num_visual_geometries(), 11);
}

TEST_F(UsdParserTest, NoSuchFile) {
  std::string filename = "/no/such/file";
  const DataSource source{DataSource::kFilename, &filename};
  ParseFile(source);
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(".*File does not exist.*"));
}

TEST_F(UsdParserTest, InvalidFileTest) {
  std::string filename = FindUsdTestResourceOrThrow("invalid/invalid_file.usd");
  const DataSource source{DataSource::kFilename, &filename};
  ParseFile(source);
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*Failed to open USD stage:.*"));
}

TEST_F(UsdParserTest, InvalidInMemoryStageTest) {
  std::string file_content = R"""(Invalid USD File})""";
  const DataSource source{DataSource::kContents, &file_content};
  ParseFile(source);
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*Failed to load in-memory USD stage."));
}

TEST_F(UsdParserTest, MissingStageMetadataTest) {
  std::string file_content = R"""(#usda 1.0
    def "SomePrim" { })""";
  const DataSource source{DataSource::kContents, &file_content};
  ParseFile(source);
  EXPECT_THAT(TakeWarning(),
              ::testing::MatchesRegex(
                  ".*Failed to read metersPerUnit in stage metadata.*"));
  EXPECT_THAT(
      TakeWarning(),
      ::testing::MatchesRegex(".*Failed to read upAxis in stage metadata.*"));
}

TEST_F(UsdParserTest, UnsupportedPrimTypesTest) {
  std::string file_content = R"""(#usda 1.0
    (
      metersPerUnit = 1
      upAxis = "Z"
    )
    def Xform "World"
    {
      def "Box" (prepend apiSchemas = ["PhysicsCollisionAPI"]) { }
      def Cone "Cone" (prepend apiSchemas = ["PhysicsCollisionAPI"]) { }
    })""";
  const DataSource source{DataSource::kContents, &file_content};
  ParseFile(source);
  // Errors from the `/World/Box` prim.
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(
                  ".*The type of the Prim at /World/Box is not specified.*"));
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
                               ".*Failed to create collision geometry.*"));
  // Errors from the `/World/Cone` Prim.
  EXPECT_THAT(TakeError(),
              ::testing::MatchesRegex(".*Unsupported Prim type 'Cone'.*"));
  EXPECT_THAT(TakeError(), ::testing::MatchesRegex(
                               ".*Failed to create collision geometry.*"));
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
