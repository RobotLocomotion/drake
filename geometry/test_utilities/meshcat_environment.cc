#include "drake/geometry/test_utilities/meshcat_environment.h"

#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"

namespace drake {
namespace geometry {

namespace {

class MeshcatEnvironment final : public ::testing::Environment {
 public:
  // This function is called by googletest during main(), but before any test
  // cases are run.
  void SetUp() final { meshcat_ = std::make_shared<Meshcat>(); }

  // This function is called by googletest during main() after all test cases
  // have been run.
  void TearDown() final { meshcat_.reset(); }

  // Helper for GetTestEnvironmentMeshcat, below.
  std::shared_ptr<Meshcat> GetSingleton() const {
    DRAKE_DEMAND(meshcat_ != nullptr);
    return meshcat_;
  }

 private:
  std::shared_ptr<Meshcat> meshcat_;
};

// This is a global variable with a non-trivial constructor, so is a violation
// of our style guide rules related to the static initialization order fiasco.
// However, we know that it will be safe because it's constructor does not
// depend on any other code, and unfortunately it is difficult to formulate
// testing globals that need tear-down in any other way.
const ::testing::Environment* g_meshcat_environment =
    testing::AddGlobalTestEnvironment(new MeshcatEnvironment);

}  // namespace

std::shared_ptr<Meshcat> GetTestEnvironmentMeshcat() {
  DRAKE_DEMAND(g_meshcat_environment != nullptr);
  auto* cast = dynamic_cast<const MeshcatEnvironment*>(g_meshcat_environment);
  DRAKE_DEMAND(cast != nullptr);
  return cast->GetSingleton();
}

}  // namespace geometry
}  // namespace drake
