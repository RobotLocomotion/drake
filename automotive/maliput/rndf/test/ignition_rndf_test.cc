#include "gtest/gtest.h"
#include "ignition/rndf/RNDF.hh"

namespace drake {
namespace maliput {
namespace rndf {

// Exercise the Bazel infrastructure and ensure that Ignition RNDF can be
// built and linked.
GTEST_TEST(IgnitionRNDFTest, Test) {
  ignition::rndf::RNDF rndfLoader;
  EXPECT_FALSE(rndfLoader.Valid());
};

}  // namespace rndf
}  // namespace maliput
}  // namespace drake
