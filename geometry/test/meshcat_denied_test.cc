#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/geometry/meshcat.h"

namespace drake {
namespace geometry {
namespace {

GTEST_TEST(MeshcatDeniedTest, ExceptionMessage) {
  DRAKE_EXPECT_THROWS_MESSAGE(Meshcat{}, ".*DRAKE_ALLOW_NETWORK.*");
}

}  // namespace
}  // namespace geometry
}  // namespace drake
