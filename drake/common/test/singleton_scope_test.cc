#include "drake/common/singleton_scope.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace drake {

namespace {

using std::make_shared;
using std::shared_ptr;

/*
 * This test verifies that the SingletonScope will scope access to a resource.
 */
int global_counter = 0;

/*
 * Simple resource to change a global counter when it is acquired / released.
 */
class Resource {
 public:
  Resource()
    : value_(++global_counter) {}
  ~Resource() {
    --global_counter;
  }
  int value() const { return value_; }
 private:
  int value_{};
};

/*
 * Incorporate expectations into resource lock class.
 */
class ResourceScope {
 public:
  explicit ResourceScope(int use_count_expected)
      : ResourceScope(use_count_expected, use_count_expected) {}

  ResourceScope(int use_count_ctor_expected, int use_count_dtor_expected,
               int value_expected = 1)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(value_expected, singleton_scope_.instance().value());
    EXPECT_EQ(use_count_ctor_expected, singleton_scope_.use_count());
  }

  ~ResourceScope() {
    EXPECT_EQ(use_count_dtor_expected_, singleton_scope_.use_count());
  }

 private:
  SingletonScope<Resource> singleton_scope_;
  int use_count_dtor_expected_;
};

/*
 * Test neatly nested locks.
 */
GTEST_TEST(SingletonScopeTest, NestedTest) {
  EXPECT_EQ(0, global_counter);
  {
    ResourceScope first(1);
    EXPECT_EQ(1, global_counter);
    {
      ResourceScope second(2);
      EXPECT_EQ(1, global_counter);
      {
        ResourceScope third(3);
        EXPECT_EQ(1, global_counter);
      }
      EXPECT_EQ(1, global_counter);
    }
    EXPECT_EQ(1, global_counter);
  }
  EXPECT_EQ(0, global_counter);
}

/*
 * Test ragged destruction with nesting mixed in.
 */
GTEST_TEST(SingletonScopeTest, RaggedTest) {
  EXPECT_EQ(0, global_counter);

  {
    auto first = make_shared<ResourceScope>(1, 2);
    EXPECT_EQ(1, global_counter);

    auto second = make_shared<ResourceScope>(2, 3);
    EXPECT_EQ(1, global_counter);

    auto third = make_shared<ResourceScope>(3, 1);
    EXPECT_EQ(1, global_counter);

    {
      ResourceScope fourth(4);
      EXPECT_EQ(1, global_counter);
    }
    EXPECT_EQ(1, global_counter);

    second.reset();
    EXPECT_EQ(1, global_counter);

    first.reset();
    EXPECT_EQ(1, global_counter);

    // Let third be implicitly destroyed
  }

  EXPECT_EQ(0, global_counter);
}

/*
 * Same as ResourceScope, but provide ability to specialize template definition,
 * such that we can maintain multiple singletons of type Resource.
 */
template <typename Parent>
class SpecializedResourceScope {
 public:
  explicit SpecializedResourceScope(int use_count_expected)
      : SpecializedResourceScope(use_count_expected, use_count_expected) {}

  SpecializedResourceScope(int use_count_ctor_expected,
                          int use_count_dtor_expected,
                          int value_expected = 1)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(value_expected, singleton_scope_.instance().value());
    EXPECT_EQ(use_count_ctor_expected, singleton_scope_.use_count());
  }

  ~SpecializedResourceScope() {
    EXPECT_EQ(use_count_dtor_expected_, singleton_scope_.use_count());
  }

 private:
  SingletonScope<Resource, Parent> singleton_scope_;
  int use_count_dtor_expected_;
};

struct A {};
struct B {};

/*
 * Test unique specializations for a Resource singleton.
 */
GTEST_TEST(SingletonScopeTest, SpecializationTest) {
  EXPECT_EQ(0, global_counter);

  // These will be the same since it uses `void` (the default Parent argument).
  {
    SpecializedResourceScope<void> first(1);
    EXPECT_EQ(1, global_counter);
    ResourceScope second(2);
    EXPECT_EQ(1, global_counter);
  }
  EXPECT_EQ(0, global_counter);

  // This will be different.
  // Note that each singleton will record the value of global_counter:
  //   ResourceScope.instance().value == 1
  //   SpecializedResourceScope<A>.instance().value == 2
  //   SpecializedResourceScope<B>.instance().value == 3
  const int counter_normal = 1;
  const int counter_a = 2;
  const int counter_b = 3;
  {
    ResourceScope normal_1(1, 1, counter_normal);
    EXPECT_EQ(1, global_counter);

    {
      SpecializedResourceScope<A> a_1(1, 1, counter_a);
      EXPECT_EQ(2, global_counter);

      // Will refer to singleton value
      ResourceScope normal_2(2, 2, counter_normal);
      // Global value still remains
      EXPECT_EQ(2, global_counter);

      {
        SpecializedResourceScope<B> b_1(1, 1, counter_b);
        EXPECT_EQ(3, global_counter);

        SpecializedResourceScope<A> a_2(2, 2, counter_a);
        EXPECT_EQ(3, global_counter);

        SpecializedResourceScope<B> b_2(2, 2, counter_b);
        EXPECT_EQ(3, global_counter);
      }
      EXPECT_EQ(2, global_counter);
    }
    EXPECT_EQ(1, global_counter);
  }
  EXPECT_EQ(0, global_counter);
}

}  // anonymous namespace
}  // namespace drake
