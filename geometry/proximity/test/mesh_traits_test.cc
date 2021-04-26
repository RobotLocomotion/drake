#include "drake/geometry/proximity/mesh_traits.h"

#include <type_traits>

#include <gtest/gtest.h>

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(PromoteNumerical, Promotion) {
  static_assert(std::is_same_v<promoted_numerical_t<double, double>, double>,
                "Double-double should be double");
  static_assert(
      std::is_same_v<promoted_numerical_t<double, AutoDiffXd>, AutoDiffXd>,
      "double-AutoDiffXd should be AutoDiffXd");
  static_assert(
      std::is_same_v<promoted_numerical_t<AutoDiffXd, double>, AutoDiffXd>,
      "AutoDiffXd-double should be AutoDiffXd");
  static_assert(
      std::is_same_v<promoted_numerical_t<AutoDiffXd, AutoDiffXd>, AutoDiffXd>,
      "AutoDiffXd-AutoDiffXd should be AutoDiffXd");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
