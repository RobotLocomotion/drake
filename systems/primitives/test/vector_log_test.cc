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
  static constexpr int64_t kDefaultCapacity = VectorLog<T>::kDefaultCapacity;
  VectorLog<T> log_{3};
  VectorX<T> record_{Vector3<T>{1.1, 2.2, 3.3}};
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

  log.Clear();
  EXPECT_EQ(log.num_samples(), 0);
}

TYPED_TEST(VectorLogFixture, Move) {
  auto& log = this->log_;
  auto& record = this->record_;
  log.AddData(0.001, record);
  EXPECT_EQ(log.num_samples(), 1);

  auto other_log = std::move(log);
  EXPECT_EQ(other_log.num_samples(), 1);
  // The moved-from log becomes empty.
  EXPECT_EQ(log.num_samples(), 0);
}

TYPED_TEST(VectorLogFixture, GrowLog) {
  auto& log = this->log_;
  auto& record = this->record_;
  auto goal_size = 1 + this->kDefaultCapacity;

  for (int k = 0; k < goal_size; k++) {
    log.AddData(0.001 + k, record);
  }
  EXPECT_EQ(log.num_samples(), goal_size);
  EXPECT_EQ(log.sample_times()[goal_size - 1], 0.001 + goal_size - 1);
  EXPECT_EQ(log.data()(0, goal_size - 1), 1.1);
  EXPECT_EQ(log.data()(1, goal_size - 1), 2.2);
  EXPECT_EQ(log.data()(2, goal_size - 1), 3.3);
}

}  // namespace
}  // namespace systems
}  // namespace drake
