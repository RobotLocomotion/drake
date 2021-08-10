#include "package1/package2/package2.hpp"
#include <gtest/gtest.h>

namespace {

GTEST_TEST(Foo, Foo) {
  package1::package2::lcmt_foo foo{};
  foo.bar.value = 5;
  EXPECT_GT(foo.getEncodedSize(), 0);
}

}  // namespace
