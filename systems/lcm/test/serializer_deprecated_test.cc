#include <gtest/gtest.h>

#include "drake/common/is_cloneable.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/lcm/serializer.h"

namespace drake {
namespace systems {
namespace lcm {
namespace {

GTEST_TEST(SerializerDeprecatedTest, BasicTest) {
  // The device under test.
  auto dut = std::make_unique<Serializer<lcmt_drake_signal>>();

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  // Cloning works.
  EXPECT_TRUE(is_cloneable<SerializerInterface>::value);
  auto fresh = dut->Clone();
  ASSERT_NE(fresh, nullptr);
  auto fresh_value = fresh->CreateDefaultValue();
  EXPECT_EQ(fresh_value->get_value<lcmt_drake_signal>().dim, 0);
#pragma GCC diagnostic pop
}

}  // namespace
}  // namespace lcm
}  // namespace systems
}  // namespace drake
