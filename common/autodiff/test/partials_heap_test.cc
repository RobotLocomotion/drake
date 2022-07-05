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
class PartialsHeapTest : public ::testing::Test {
 protected:
};

TEST_F(PartialsHeapTest, TODO) {
}

}  // namespace
}  // namespace autodiff
}  // namespace drake
