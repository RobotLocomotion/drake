#include "drake/common/test_utilities/is_dynamic_castable.h"

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "drake/common/nice_type_name.h"

namespace drake {
namespace {
struct A {
  virtual ~A() {}
};
struct B {
  virtual ~B() {}
};
struct C : A, B {
  ~C() override {}
};

using std::make_shared;
using std::shared_ptr;
using std::string;

class IsDynamicCastableTest : public ::testing::Test {
 protected:
  const shared_ptr<A> a_a_{make_shared<A>()};
  const shared_ptr<A> a_c_{make_shared<C>()};
  const shared_ptr<B> b_b_{make_shared<B>()};
  const shared_ptr<B> b_c_{make_shared<C>()};
  const shared_ptr<C> c_c_{make_shared<C>()};
};

TEST_F(IsDynamicCastableTest, RawPointer) {
  EXPECT_TRUE(is_dynamic_castable<A>(a_a_.get()));
  EXPECT_FALSE(is_dynamic_castable<B>(a_a_.get()));
  EXPECT_FALSE(is_dynamic_castable<C>(a_a_.get()));

  EXPECT_TRUE(is_dynamic_castable<A>(a_c_.get()));
  EXPECT_TRUE(is_dynamic_castable<B>(a_c_.get()));  // sidecasting A -> B
  EXPECT_TRUE(is_dynamic_castable<C>(a_c_.get()));  // downcasting A -> C

  EXPECT_FALSE(is_dynamic_castable<A>(b_b_.get()));
  EXPECT_TRUE(is_dynamic_castable<B>(b_b_.get()));
  EXPECT_FALSE(is_dynamic_castable<C>(b_b_.get()));

  EXPECT_TRUE(is_dynamic_castable<A>(b_c_.get()));  // sidecasting B -> A
  EXPECT_TRUE(is_dynamic_castable<B>(b_c_.get()));
  EXPECT_TRUE(is_dynamic_castable<C>(b_c_.get()));  // downcasting B -> C

  EXPECT_TRUE(is_dynamic_castable<A>(c_c_.get()));  // upcasting C -> A
  EXPECT_TRUE(is_dynamic_castable<B>(c_c_.get()));  // upcasting C -> B
  EXPECT_TRUE(is_dynamic_castable<C>(c_c_.get()));
}

TEST_F(IsDynamicCastableTest, SharedPtr) {
  EXPECT_TRUE(is_dynamic_castable<A>(a_a_));
  EXPECT_FALSE(is_dynamic_castable<B>(a_a_));
  EXPECT_FALSE(is_dynamic_castable<C>(a_a_));

  EXPECT_TRUE(is_dynamic_castable<A>(a_c_));
  EXPECT_TRUE(is_dynamic_castable<B>(a_c_));  // sidecasting A -> B
  EXPECT_TRUE(is_dynamic_castable<C>(a_c_));  // downcasting A -> C

  EXPECT_FALSE(is_dynamic_castable<A>(b_b_));
  EXPECT_TRUE(is_dynamic_castable<B>(b_b_));
  EXPECT_FALSE(is_dynamic_castable<C>(b_b_));

  EXPECT_TRUE(is_dynamic_castable<A>(b_c_));  // sidecasting B -> A
  EXPECT_TRUE(is_dynamic_castable<B>(b_c_));
  EXPECT_TRUE(is_dynamic_castable<C>(b_c_));  // downcasting B -> C

  EXPECT_TRUE(is_dynamic_castable<A>(c_c_));  // upcasting C -> A
  EXPECT_TRUE(is_dynamic_castable<B>(c_c_));  // upcasting C -> B
  EXPECT_TRUE(is_dynamic_castable<C>(c_c_));
}

string ExpectedMessage(const string& derived, const string& base,
                       const string& dynamic) {
  return "is_dynamic_castable<" + derived + ">(" + base +
         "* ptr) failed because ptr is of dynamic type " + dynamic + ".";
}

TEST_F(IsDynamicCastableTest, DiagnosticMessage) {
  const string a_typename{NiceTypeName::Get<A>()};
  const string b_typename{NiceTypeName::Get<B>()};
  const string c_typename{NiceTypeName::Get<C>()};

  const auto assertion_result1 = is_dynamic_castable<B>(a_a_);
  ASSERT_FALSE(assertion_result1);
  EXPECT_EQ(assertion_result1.failure_message(),
            ExpectedMessage(b_typename, a_typename, a_typename));

  const auto assertion_result2 = is_dynamic_castable<C>(a_a_);
  ASSERT_FALSE(assertion_result2);
  EXPECT_EQ(assertion_result2.failure_message(),
            ExpectedMessage(c_typename, a_typename, a_typename));

  const auto assertion_result3 = is_dynamic_castable<A>(b_b_);
  ASSERT_FALSE(assertion_result3);
  EXPECT_EQ(assertion_result3.failure_message(),
            ExpectedMessage(a_typename, b_typename, b_typename));

  const auto assertion_result4 = is_dynamic_castable<C>(b_b_);
  ASSERT_FALSE(assertion_result4);
  EXPECT_EQ(assertion_result4.failure_message(),
            ExpectedMessage(c_typename, b_typename, b_typename));
}

}  // namespace
}  // namespace drake
