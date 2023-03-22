#include "drake/common/network_policy.h"

#include <stdlib.h>

#include <gtest/gtest.h>

namespace drake {
namespace internal {
namespace {

constexpr const char kAllow[] = "DRAKE_ALLOW_NETWORK";

GTEST_TEST(NetworkPolicyTest, Missing) {
  ASSERT_EQ(::unsetenv(kAllow), 0);
  EXPECT_TRUE(IsNetworkingAllowed("foo"));
}

GTEST_TEST(NetworkPolicyTest, Empty) {
  ASSERT_EQ(::setenv(kAllow, "", 1), 0);
  EXPECT_TRUE(IsNetworkingAllowed("foo"));
}

GTEST_TEST(NetworkPolicyTest, None) {
  ASSERT_EQ(::setenv(kAllow, "none", 1), 0);
  EXPECT_FALSE(IsNetworkingAllowed("foo"));
}

GTEST_TEST(NetworkPolicyTest, MixedNone) {
  // N.B. This prints a spdlog warning.
  ASSERT_EQ(::setenv(kAllow, "none:foo", 1), 0);
  EXPECT_FALSE(IsNetworkingAllowed("foo"));
}

GTEST_TEST(NetworkPolicyTest, YesMatch) {
  ASSERT_EQ(::setenv(kAllow, "foo", 1), 0);
  EXPECT_TRUE(IsNetworkingAllowed("foo"));

  ASSERT_EQ(::setenv(kAllow, "foo:bar", 1), 0);
  EXPECT_TRUE(IsNetworkingAllowed("foo"));

  ASSERT_EQ(::setenv(kAllow, "bar:foo:baz", 1), 0);
  EXPECT_TRUE(IsNetworkingAllowed("foo"));
}

GTEST_TEST(NetworkPolicyTest, NoMatch) {
  ASSERT_EQ(::setenv(kAllow, "bar", 1), 0);
  EXPECT_FALSE(IsNetworkingAllowed("foo"));

  ASSERT_EQ(::setenv(kAllow, "bar:baz", 1), 0);
  EXPECT_FALSE(IsNetworkingAllowed("foo"));

  ASSERT_EQ(::setenv(kAllow, ":bar:baz:", 1), 0);
  EXPECT_FALSE(IsNetworkingAllowed("foo"));
}

GTEST_TEST(NetworkPolicyTest, BadName) {
  EXPECT_THROW(IsNetworkingAllowed(""), std::exception);
  EXPECT_THROW(IsNetworkingAllowed("none"), std::exception);
  EXPECT_THROW(IsNetworkingAllowed("LCM"), std::exception);
}

}  // namespace
}  // namespace internal
}  // namespace drake
