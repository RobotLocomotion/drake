#include "drake/geometry/test_utilities/meshcat_environment.h"

#include <chrono>
#include <thread>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "drake/common/drake_assert.h"
#include "drake/common/scope_exit.h"
#include "drake/common/text_logging.h"

DEFINE_bool(
    meshcat_pause_during_tests, false,
   "When running the tests, pause for the user clicks, for debugging. This"
    " is probably most useful when also using --gtest_filter for brevity.");

namespace drake {
namespace geometry {

namespace {

class MeshcatEnvironment final : public ::testing::Environment {
 public:
  // This function is called by googletest during main(), but before any test
  // cases are run.
  void SetUp() final {
    meshcat_ = std::make_shared<Meshcat>();
  }

  // This function is called by googletest during main() after all test cases
  // have been run.
  void TearDown() final {
    meshcat_.reset();
  }

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

void PauseTestEnvironmentMeshcat() {
  std::shared_ptr<Meshcat> meshcat = GetTestEnvironmentMeshcat();

  const std::string label = fmt::format(
      "Paused test {}, click to continue",
      ::testing::UnitTest::GetInstance()->current_test_info()->name());
  meshcat->AddButton(label);
  ScopeExit guard([&meshcat, label]() {
    meshcat->DeleteButton(label);
  });

  if (FLAGS_meshcat_pause_during_tests) {
    drake::log()->info(
        "Pausing for the 'click here' button on the Meshcat control panel");
    while (meshcat->GetButtonClicks(label) == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

}  // namespace geometry
}  // namespace drake

