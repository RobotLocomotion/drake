#include "drake/lcm/drake_lcm_interface.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "drake/lcm/drake_mock_lcm.h"
#include "drake/lcm/lcmt_drake_signal_utils.h"
#include "drake/lcmt_drake_signal.hpp"

namespace drake {
namespace lcm {
namespace {

GTEST_TEST(DrakeLcmInterfaceTest, PublishTest) {
  // Publish using the helper function.
  DrakeMockLcm lcm;
  const std::string name = "NAME";
  lcmt_drake_signal original{};
  original.timestamp = 123;
  Publish(&lcm, name, original);

  // Make sure it came out okay.  (Manually decode so as to be slightly less
  // dependent on the DrakeMockLcm sugar.)
  const std::vector<uint8_t>& bytes = lcm.get_last_published_message(name);
  lcmt_drake_signal decoded{};
  EXPECT_EQ(decoded.decode(bytes.data(), 0, bytes.size()), bytes.size());
  EXPECT_TRUE(CompareLcmtDrakeSignalMessages(decoded, original));
}

}  // namespace
}  // namespace lcm
}  // namespace drake
