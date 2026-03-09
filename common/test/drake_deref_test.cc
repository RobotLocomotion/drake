#include <memory>

#include <gtest/gtest.h>

#include "drake/common/drake_throw.h"
#include "drake/common/test_utilities/expect_no_throw.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace {

GTEST_TEST(DrakeDerefTest, Acceptance) {
  // Raw pointers.
  int x = 42;
  int* p1 = &x;
  int* const p2 = &x;
  const int* p3 = &x;
  const int* const p4 = &x;
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(p1));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(p2));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(p3));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(p4));

  // To pass a boolean to gtest that comes from a type trait value that wraps a
  // macro, we need to launder it through parentheses (hence the apparently
  // superfluous parentheses).

  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(p1)), int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(p2)), int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(p3)), const int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(p4)), const int&>));

  // Unique pointers.
  std::unique_ptr<int> up1 = std::make_unique<int>();
  const std::unique_ptr<int> up2 = std::make_unique<int>();
  const std::unique_ptr<const int> up3 = std::make_unique<int>();
  std::unique_ptr<const int> up4 = std::make_unique<int>();
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(up1));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(up2));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(up3));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(up4));

  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(up1)), int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(up2)), int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(up3)), const int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(up4)), const int&>));

  // Shared pointers.
  std::shared_ptr<int> sp1 = std::make_shared<int>();
  const std::shared_ptr<int> sp2 = std::make_shared<int>();
  const std::shared_ptr<const int> sp3 = std::make_shared<int>();
  std::shared_ptr<const int> sp4 = std::make_shared<int>();
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(sp1));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(sp2));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(sp3));
  DRAKE_EXPECT_NO_THROW(DRAKE_DEREF(sp4));

  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(sp1)), int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(sp2)), int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(sp3)), const int&>));
  EXPECT_TRUE((std::is_same_v<decltype(DRAKE_DEREF(sp4)), const int&>));
}

GTEST_TEST(DrakeDerefTest, RejectNull) {
  int* p_null = nullptr;
  DRAKE_EXPECT_THROWS_MESSAGE(DRAKE_DEREF(p_null),
                              ".*condition 'p_null != nullptr' failed.*");
  std::unique_ptr<int> up_null = nullptr;
  DRAKE_EXPECT_THROWS_MESSAGE(DRAKE_DEREF(up_null),
                              ".*condition 'up_null != nullptr' failed.*");
  std::shared_ptr<int> sp_null = nullptr;
  DRAKE_EXPECT_THROWS_MESSAGE(DRAKE_DEREF(sp_null),
                              ".*condition 'sp_null != nullptr' failed.*");
}

}  // namespace
