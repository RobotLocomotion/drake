#include <gtest/gtest.h>

#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/lcm/drake_lcm.h"

namespace drake {
namespace lcm {
namespace {

GTEST_TEST(DrakeLcmDeniedTest, MemqIsFine) {
  EXPECT_NO_THROW(DrakeLcm{"memq://foo"});
}

GTEST_TEST(DrakeLcmDeniedTest, ExceptionMessage) {
  DRAKE_EXPECT_THROWS_MESSAGE(DrakeLcm{"udpm://239.255.76.67:7670"},
                              ".*DRAKE_ALLOW_NETWORK.*");
}

}  // namespace
}  // namespace lcm
}  // namespace drake
