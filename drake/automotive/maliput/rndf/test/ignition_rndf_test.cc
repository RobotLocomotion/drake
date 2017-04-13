#include "gtest/gtest.h"
#include "ignition/rndf/RNDF.hh"

namespace drake {
namespace maliput {
namespace monolane {

GTEST_TEST(IgnitionRNDFTest, Test) {
  ignition::rndf::RNDF rndfLoader;
  EXPECT_FALSE(rndfLoader.Valid());
};


}  // namespace monolane
}  // namespace maliput
}  // namespace drake
