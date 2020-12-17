#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/eigen_types.h"
#include "drake/common/test_utilities/limit_malloc.h"

using Eigen::VectorXd;

namespace drake {
namespace test {
namespace {

// Provide some autodiff variables that don't expire at scope end for test
// cases.
class AutoDiffXdHeapTest : public ::testing::Test {
 protected:
  AutoDiffXd x_{0.4, Eigen::VectorXd::Ones(3)};
  AutoDiffXd y_{0.3, Eigen::VectorXd::Ones(3)};
};

// @note The test cases use a sum in function arguments to induce a temporary
// that can potentially be consumed by pass-by-value optimizations. As of this
// writing, none of the tested functions uses the optimization, so current heap
// counts are a baseline for the existing implementations.

// @note The tests cases use `volatile` variables to prevent optimizers from
// removing the tested function calls entirely. As of this writing, there is no
// evidence that the technique is strictly necessary. However, future
// implementations may be vulnerable to dead-code elimination.

TEST_F(AutoDiffXdHeapTest, Abs) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = abs(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Abs2) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = abs2(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Acos) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = acos(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Asin) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = asin(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Atan) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = atan(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Atan2) {
  {
    LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
    volatile auto v = atan2(x_ + y_, y_);
  }
  {
    LimitMalloc guard({.max_num_allocations = 2, .min_num_allocations = 2});
    // Right-hand parameter moves are blocked by code in Eigen; see #14039.
    volatile auto v = atan2(y_, x_ + y_);
  }
}

TEST_F(AutoDiffXdHeapTest, Cos) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = cos(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Cosh) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = cosh(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Exp) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = exp(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Log) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = log(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Min) {
  LimitMalloc guard({.max_num_allocations = 4, .min_num_allocations = 4});
  volatile auto v = min(x_ + y_, y_);
  volatile auto w = min(x_, x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Max) {
  LimitMalloc guard({.max_num_allocations = 4, .min_num_allocations = 4});
  volatile auto v = max(x_ + y_, y_);
  volatile auto w = max(x_, x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Pow) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = pow(x_ + y_, 2.0);
}

TEST_F(AutoDiffXdHeapTest, Sin) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = sin(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Sinh) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = sinh(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Sqrt) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = sqrt(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Tan) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = tan(x_ + y_);
}

TEST_F(AutoDiffXdHeapTest, Tanh) {
  LimitMalloc guard({.max_num_allocations = 1, .min_num_allocations = 1});
  volatile auto v = tanh(x_ + y_);
}

}  // namespace
}  // namespace test
}  // namespace drake
