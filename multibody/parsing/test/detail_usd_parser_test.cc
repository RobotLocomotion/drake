#include "drake/multibody/parsing/detail_usd_parser.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/diagnostic_policy_test_base.h"
#include "drake/common/test_utilities/expect_throws_message.h"

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

// TODO(jwnimmer-tri) This is a very basic sanity test, just to get the ball
// rolling. It spews lots of error messages that probably indicate deeper
// problems. But for now, it passes!
TEST_F(UsdParserTest, Stub) {
  const fs::path filename{"no_such_file.usda"};
  DRAKE_EXPECT_THROWS_MESSAGE(ParseFile(filename),
                              ".*UsdParser.*AddAllModels.*not implemented.*");
}

}  // namespace
}  // namespace internal
}  // namespace multibody
}  // namespace drake
