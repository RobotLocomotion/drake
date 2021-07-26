#include "drake/systems/primitives/vector_log.h"

#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/symbolic.h"

namespace drake {
namespace systems {
namespace {

template <typename T>
class VectorLogFixture : public testing::Test {
 protected:
  VectorLog<T> log_{3};
  Vector3<T> record_{1.1, 2.2, 3.3};
};

using ScalarTypes = ::testing::Types<double, AutoDiffXd, symbolic::Expression>;

TYPED_TEST_SUITE(VectorLogFixture, ScalarTypes);

TYPED_TEST(VectorLogFixture, Basics) {
  auto& log = this->log_;
  auto& record = this->record_;
  EXPECT_EQ(log.get_input_size(), 3);
  EXPECT_EQ(log.num_samples(), 0);

  log.AddData(0.001, record);
  EXPECT_EQ(log.num_samples(), 1);
  EXPECT_EQ(log.sample_times()[0], 0.001);
  EXPECT_EQ(log.data()(0, 0), 1.1);
  EXPECT_EQ(log.data()(1, 0), 2.2);
  EXPECT_EQ(log.data()(2, 0), 3.3);

  log.Reset();
  EXPECT_EQ(log.num_samples(), 0);
}

}  // namespace
}  // namespace systems
}  // namespace drake
