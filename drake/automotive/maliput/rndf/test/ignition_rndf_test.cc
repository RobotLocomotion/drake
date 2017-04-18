#include "gtest/gtest.h"
#include "ignition/rndf/RNDF.hh"

namespace drake {
namespace maliput {
namespace rndf {

GTEST_TEST(IgnitionRNDFTest, Test) {
  ignition::rndf::RNDF rndfLoader;
  EXPECT_FALSE(rndfLoader.Valid());
};


}  // namespace rndf
}  // namespace maliput
}  // namespace drake
