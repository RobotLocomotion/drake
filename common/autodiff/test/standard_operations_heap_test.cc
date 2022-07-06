#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/limit_malloc.h"

namespace drake {
namespace autodiff {
namespace {

using Eigen::VectorXd;
using test::LimitMalloc;

// Provide some autodiff variables that don't expire at scope end for test
// cases.
class HeapTest : public ::testing::Test {
 protected:
  AutoDiff x_{0.4, Eigen::VectorXd::Ones(3)};
  AutoDiff y_{0.3, Eigen::VectorXd::Ones(3)};
};

// @note The test cases use a sum in function arguments to induce a temporary
// that can potentially be consumed by pass-by-value optimizations. As of this
// writing, none of the tested functions uses the optimization, so current heap
// counts are a baseline for the existing implementations.

// @note The tests cases use `volatile` variables to prevent optimizers from
// removing the tested function calls entirely. As of this writing, there is no
// evidence that the technique is strictly necessary. However, future
// implementations may be vulnerable to dead-code elimination.

TEST_F(HeapTest, Abs) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 0});
  volatile auto v = abs(x_ + y_);
  (void)(v);
}

#if 0
TEST_F(HeapTest, Abs2) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = abs2(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Acos) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = acos(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Asin) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = asin(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Atan) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = atan(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Atan2) {
  {
    LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
    volatile auto v = atan2(x_ + y_, y_);
    (void)(v);
  }
  {
    LimitMalloc guard({.max_num_allocations = 2, .min_num_allocations = 2});
    // Right-hand parameter moves are blocked by code in Eigen; see #14039.
    volatile auto v = atan2(y_, x_ + y_);
    (void)(v);
  }
}

TEST_F(HeapTest, Cos) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = cos(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Cosh) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = cosh(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Exp) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = exp(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Log) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = log(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Min) {
  LimitMalloc guard({.max_num_allocations = 3, .min_num_allocations = 3});
  volatile auto v = min(x_ + y_, y_);
  volatile auto w = min(x_, x_ + y_);
  (void)(v);
  (void)(w);
}

TEST_F(HeapTest, Max) {
  LimitMalloc guard({.max_num_allocations = 3, .min_num_allocations = 3});
  volatile auto v = max(x_ + y_, y_);
  volatile auto w = max(x_, x_ + y_);
  (void)(v);
  (void)(w);
}

TEST_F(HeapTest, Pow) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = pow(x_ + y_, 2.0);
  (void)(v);
}

TEST_F(HeapTest, Sin) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = sin(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Sinh) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = sinh(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Sqrt) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = sqrt(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Tan) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = tan(x_ + y_);
  (void)(v);
}

TEST_F(HeapTest, Tanh) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = tanh(x_ + y_);
  (void)(v);
}
#endif

}  // namespace
}  // namespace autodiff
}  // namespace drake
