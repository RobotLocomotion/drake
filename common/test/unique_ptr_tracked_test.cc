#include "drake/common/unique_ptr_tracked.h"

#include <gtest/gtest.h>

using std::unique_ptr;

namespace drake {

int num_alive = 0;

struct Value {
  Value() {
    num_alive++;
  }
  ~Value() {
    num_alive--;
  }
  Value(const Value&) = delete;
  Value(Value&&) = delete;
};

using internal::on_delete;
using internal::PtrErased;

class UniquePtrTracked : public ::testing::Test {
 public:
  void SetUp() virtual {
    ASSERT_FALSE(on_delete);
  }
  void TearDown() virtual {
    on_delete = DeleteCallback{};
  }
};

TEST_F(UniquePtrTracked, TestNominal) {
  // Ensure that nominal behavior is guaranteed without a callback installed.
  unique_ptr<Value> ptr;
  EXPECT_EQ(num_alive, 1);
  ptr.reset();
  EXPECT_EQ(num_alive, 0);
}

TEST_F(UniquePtrTracked, TestWithCallback) {
  // Test nominal behavior.
  PtrErased ptr_last;
  on_delete = [&ptr_last](PtrErased ptr) {
    // Just store the last value.
    ptr_last = ptr;
    return false;
  };

  unique_ptr<Value> ptr(new Value());
  Value* raw = ptr.get();
  EXPECT_EQ(num_alive, 1);
  {
    unique_ptr_tracked<Value> ptr_tracked = std::move(ptr);
    // Test moving back and forth.
    unique_ptr<Value> ptr_2 = std::move(ptr_tracked);
    ptr_tracked = std::move(ptr_2);
    ptr = std::move(ptr_tracked);
  }
  // Lifetime should not have ended.
  ASSERT_FALSE(ptr_last);
  {
    // The lifetime should end here.
    unique_ptr_tracked<Value> ptr_tracked_2;
    ptr_tracked_2 = std::move(ptr);
  }
  EXPECT_EQ(num_alive, 0);
  EXPECT_EQ(raw, ptr_last.ptr());
}

}  // namespace drake
