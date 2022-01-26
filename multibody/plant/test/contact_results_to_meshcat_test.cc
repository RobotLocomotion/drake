#include "drake/multibody/plant/contact_results_to_meshcat.h"

#include <gtest/gtest.h>

namespace drake {
namespace multibody {
namespace {

// Merely confirm that the aliases can compile.
GTEST_TEST(ContactResultToMeshcatTest, DeprecatedAliases) {
  multibody::ContactResultsToMeshcatParams params;
  multibody::ContactResultsToMeshcat<double>* unused{};
  static_cast<void>(unused);
}

}  // namespace
}  // namespace multibody
}  // namespace drake
