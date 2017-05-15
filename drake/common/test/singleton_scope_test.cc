#include "drake/common/singleton_scope.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

namespace drake {

namespace {

using std::make_shared;
using std::shared_ptr;

/*
 * Simple resource to change a global counter when it is acquired / released.
 */
class TrivialSingleton {
 public:
  TrivialSingleton()
    : instance_count_at_construction_(++instance_count_) {}
  ~TrivialSingleton() {
    --instance_count_;
  }
  int instance_count_at_construction() const {
    return instance_count_at_construction_;
  }
  static int instance_count() { return instance_count_; }
 private:
  int instance_count_at_construction_{};

  static int instance_count_;
};

int TrivialSingleton::instance_count_ = 0;

/*
 * Incorporate expectations into resource lock class.
 */
class TrivialSingletonScope {
 public:
  explicit TrivialSingletonScope(int use_count_expected)
      : TrivialSingletonScope(use_count_expected, use_count_expected) {}

  TrivialSingletonScope(int use_count_ctor_expected,
                        int use_count_dtor_expected,
                        int instance_count_at_construction_expected = 1)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(instance_count_at_construction_expected,
              singleton_scope_.instance().instance_count_at_construction());
    EXPECT_EQ(use_count_ctor_expected, singleton_scope_.use_count());
  }

  ~TrivialSingletonScope() {
    EXPECT_EQ(use_count_dtor_expected_, singleton_scope_.use_count());
  }

 private:
  SingletonScope<TrivialSingleton> singleton_scope_;
  int use_count_dtor_expected_;
};

/*
 * Test neatly nested locks.
 */
GTEST_TEST(SingletonScopeTest, NestedTest) {
  EXPECT_EQ(0, TrivialSingleton::instance_count());
  {
    TrivialSingletonScope first(1);
    EXPECT_EQ(1, TrivialSingleton::instance_count());
    {
      TrivialSingletonScope second(2);
      EXPECT_EQ(1, TrivialSingleton::instance_count());
      {
        TrivialSingletonScope third(3);
        EXPECT_EQ(1, TrivialSingleton::instance_count());
      }
      EXPECT_EQ(1, TrivialSingleton::instance_count());
    }
    EXPECT_EQ(1, TrivialSingleton::instance_count());
  }
  EXPECT_EQ(0, TrivialSingleton::instance_count());
}

/*
 * Test ragged destruction with nesting mixed in.
 */
GTEST_TEST(SingletonScopeTest, RaggedTest) {
  EXPECT_EQ(0, TrivialSingleton::instance_count());

  {
    auto first = make_shared<TrivialSingletonScope>(1, 2);
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    auto second = make_shared<TrivialSingletonScope>(2, 3);
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    auto third = make_shared<TrivialSingletonScope>(3, 1);
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    {
      TrivialSingletonScope fourth(4);
      EXPECT_EQ(1, TrivialSingleton::instance_count());
    }
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    second.reset();
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    first.reset();
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    // Let third be implicitly destroyed
  }

  EXPECT_EQ(0, TrivialSingleton::instance_count());
}

/*
 * Same as TrivialSingletonScope, but provide ability to specialize template definition,
 * such that we can maintain multiple singletons of type TrivialSingleton.
 */
template <typename Parent>
class SpecializedTrivialSingletonScope {
 public:
  explicit SpecializedTrivialSingletonScope(int use_count_expected)
      : SpecializedTrivialSingletonScope(use_count_expected,
                                         use_count_expected) {}

  SpecializedTrivialSingletonScope(int use_count_ctor_expected,
                          int use_count_dtor_expected,
                          int instance_count_at_construction_expected = 1)
      : use_count_dtor_expected_(use_count_dtor_expected) {
    EXPECT_EQ(instance_count_at_construction_expected,
        singleton_scope_.instance().instance_count_at_construction());
    EXPECT_EQ(use_count_ctor_expected, singleton_scope_.use_count());
  }

  ~SpecializedTrivialSingletonScope() {
    EXPECT_EQ(use_count_dtor_expected_, singleton_scope_.use_count());
  }

 private:
  SingletonScope<TrivialSingleton, Parent> singleton_scope_;
  int use_count_dtor_expected_;
};

struct A {};
struct B {};

/*
 * Test unique specializations for a TrivialSingleton singleton.
 */
GTEST_TEST(SingletonScopeTest, SpecializationTest) {
  EXPECT_EQ(0, TrivialSingleton::instance_count());

  // These will be the same since it uses `void` (the default Parent argument).
  {
    SpecializedTrivialSingletonScope<void> first(1);
    EXPECT_EQ(1, TrivialSingleton::instance_count());
    TrivialSingletonScope second(2);
    EXPECT_EQ(1, TrivialSingleton::instance_count());
  }
  EXPECT_EQ(0, TrivialSingleton::instance_count());

  // Each singleton instance will have a different instance count at
  // construction.
  const int instance_count_normal = 1;
  const int instance_count_a = 2;
  const int instance_count_b = 3;
  {
    TrivialSingletonScope normal_1(1, 1, instance_count_normal);
    EXPECT_EQ(1, TrivialSingleton::instance_count());

    {
      SpecializedTrivialSingletonScope<A> a_1(1, 1, instance_count_a);
      EXPECT_EQ(2, TrivialSingleton::instance_count());

      // Will refer to singleton instance_count_at_construction
      TrivialSingletonScope normal_2(2, 2, instance_count_normal);
      // Global instance_count_at_construction still remains
      EXPECT_EQ(2, TrivialSingleton::instance_count());

      {
        SpecializedTrivialSingletonScope<B> b_1(1, 1, instance_count_b);
        EXPECT_EQ(3, TrivialSingleton::instance_count());

        SpecializedTrivialSingletonScope<A> a_2(2, 2, instance_count_a);
        EXPECT_EQ(3, TrivialSingleton::instance_count());

        SpecializedTrivialSingletonScope<B> b_2(2, 2, instance_count_b);
        EXPECT_EQ(3, TrivialSingleton::instance_count());
      }
      EXPECT_EQ(2, TrivialSingleton::instance_count());
    }
    EXPECT_EQ(1, TrivialSingleton::instance_count());
  }
  EXPECT_EQ(0, TrivialSingleton::instance_count());
}

}  // anonymous namespace
}  // namespace drake
