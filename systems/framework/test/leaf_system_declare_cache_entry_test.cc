#include <gtest/gtest.h>

#include "drake/systems/framework/leaf_system.h"

namespace drake {
namespace systems {
namespace {

class DummySystem : public LeafSystem<double> {};

GTEST_TEST(LeafSystemDeclareCacheEntryTest, AcceptanceTest) {
  DummySystem dummy;
  CacheEntry& entry = dummy.DeclareCacheEntry<std::string>(
      "foo_entry", std::string(),
      [](const Context<double>&, std::string* value) {
        *value = "foo";
      }, {
        dummy.nothing_ticket()
      });
  EXPECT_EQ(entry.description(), "foo_entry");
  ASSERT_EQ(entry.prerequisites().size(), 1);
  EXPECT_EQ(*entry.prerequisites().begin(), dummy.nothing_ticket());
  auto context = dummy.AllocateContext();
  EXPECT_EQ(entry.Eval<std::string>(*context), "foo");
}

}  // namespace
}  // namespace systems
}  // namespace drake
