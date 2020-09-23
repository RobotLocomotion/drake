#include "drake/systems/framework/supervector.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/vector_base.h"

namespace drake {
namespace systems {
namespace {

const int kLength = 9;

class SupervectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    vec1_ = BasicVector<double>::Make({0, 1, 2, 3});
    vec2_ = BasicVector<double>::Make({4, 5});
    vec3_ = BasicVector<double>::Make({});
    vec4_ = BasicVector<double>::Make({6, 7, 8});
    supervector_ =
        std::make_unique<Supervector<double>>(std::vector<VectorBase<double>*>{
            vec1_.get(), vec2_.get(), vec3_.get(), vec4_.get()});
  }

  std::unique_ptr<VectorBase<double>> vec1_, vec2_, vec3_, vec4_;
  std::unique_ptr<Supervector<double>> supervector_;
};

TEST_F(SupervectorTest, GetAtIndex) {
  const auto& const_supervector = *supervector_;
  ASSERT_EQ(kLength, supervector_->size());
  for (int i = 0; i < kLength; ++i) {
    EXPECT_EQ(i, supervector_->GetAtIndex(i));
    EXPECT_EQ(i, const_supervector.GetAtIndex(i));
  }
}

TEST_F(SupervectorTest, SetAtIndex) {
  for (int i = 0; i < kLength; ++i) {
    supervector_->SetAtIndex(i, i * 2);
  }
  for (int i = 0; i < kLength; ++i) {
    EXPECT_EQ(i * 2, supervector_->GetAtIndex(i));
  }

  // Confirm the changes were written through to the constituent vectors.
  EXPECT_EQ(0, vec1_->GetAtIndex(0));
  EXPECT_EQ(2, vec1_->GetAtIndex(1));
  EXPECT_EQ(4, vec1_->GetAtIndex(2));
  EXPECT_EQ(6, vec1_->GetAtIndex(3));
  EXPECT_EQ(8, vec2_->GetAtIndex(0));
  EXPECT_EQ(10, vec2_->GetAtIndex(1));
  EXPECT_EQ(12, vec4_->GetAtIndex(0));
  EXPECT_EQ(14, vec4_->GetAtIndex(1));
  EXPECT_EQ(16, vec4_->GetAtIndex(2));
}

// Tests that the Supervector can be addressed as an array.
TEST_F(SupervectorTest, ArrayOperator) {
  (*supervector_)[5] = 42;
  EXPECT_EQ(42, (*vec2_)[1]);

  const auto& const_supervector = *supervector_;
  EXPECT_EQ(8, const_supervector[8]);
}

TEST_F(SupervectorTest, OutOfRange) {
  EXPECT_THROW(supervector_->GetAtIndex(-1), std::exception);
  EXPECT_THROW(supervector_->GetAtIndex(10), std::exception);
  EXPECT_THROW(supervector_->SetAtIndex(-1, 0.0), std::exception);
  EXPECT_THROW(supervector_->SetAtIndex(10, 0.0), std::exception);
}

// Tests that a supervector can be SetFrom another vector.
TEST_F(SupervectorTest, SetFromVector) {
  auto next_value = BasicVector<double>::Make({
      10, 11, 12, 13, 14, 15, 16, 17, 18});
  supervector_->SetFromVector(next_value->CopyToVector());
  EXPECT_EQ(10, supervector_->GetAtIndex(0));
  EXPECT_EQ(11, supervector_->GetAtIndex(1));

  EXPECT_THROW(supervector_->SetFromVector(Eigen::Vector3d::Zero()),
               std::exception);
}

// Tests that a supervector can be SetFrom another VectorBase.
TEST_F(SupervectorTest, SetFrom) {
  auto next_value = BasicVector<double>::Make({
      10, 11, 12, 13, 14, 15, 16, 17, 18});
  supervector_->SetFrom(*next_value);
  EXPECT_EQ(10, supervector_->GetAtIndex(0));
  EXPECT_EQ(11, supervector_->GetAtIndex(1));

  auto bad_value = BasicVector<double>::Make(3);
  EXPECT_THROW(supervector_->SetFrom(*bad_value), std::exception);
}

TEST_F(SupervectorTest, Empty) {
  Supervector<double> supervector(std::vector<VectorBase<double>*>{});
  EXPECT_EQ(0, supervector.size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
