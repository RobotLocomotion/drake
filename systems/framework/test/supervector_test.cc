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

TEST_F(SupervectorTest, Get) {
  ASSERT_EQ(kLength, supervector_->size());
  for (int i = 0; i < kLength; ++i) {
    EXPECT_EQ(i, supervector_->at(i));
    EXPECT_EQ(i, (*supervector_)[i]);
    EXPECT_EQ(i, supervector_->GetAtIndex(i));
  }
}

TEST_F(SupervectorTest, Set) {
  for (int i = 0; i < kLength; ++i) {
    switch (i % 3) {
      case 0: { supervector_->at(i) = i * 2; break; }
      case 1: { (*supervector_)[i] = i * 2; break; }
      case 2: { supervector_->SetAtIndex(i, i * 2); break; }
    }
  }
  for (int i = 0; i < kLength; ++i) {
    // N.B. We mix up which getter is used for which setter.
    switch (i % 3) {
      case 0: { EXPECT_EQ(i * 2, supervector_->GetAtIndex(i)); break; }
      case 1: { EXPECT_EQ(i * 2, supervector_->at(i)); break; }
      case 2: { EXPECT_EQ(i * 2, (*supervector_)[i]); break; }
    }
  }

  // Confirm the changes were written through to the constituent vectors.
  EXPECT_EQ(0, vec1_->at(0));
  EXPECT_EQ(2, vec1_->at(1));
  EXPECT_EQ(4, vec1_->at(2));
  EXPECT_EQ(6, vec1_->at(3));
  EXPECT_EQ(8, vec2_->at(0));
  EXPECT_EQ(10, vec2_->at(1));
  EXPECT_EQ(12, vec4_->at(0));
  EXPECT_EQ(14, vec4_->at(1));
  EXPECT_EQ(16, vec4_->at(2));
}


// Tests that the Supervector can be addressed as an array.
TEST_F(SupervectorTest, ArrayOperator) {
  (*supervector_)[5] = 42;
  EXPECT_EQ(42, (*vec2_)[1]);
  EXPECT_EQ(8, (*supervector_)[8]);
}

TEST_F(SupervectorTest, OutOfRange) {
  EXPECT_THROW(supervector_->at(-1), std::out_of_range);
  EXPECT_THROW(supervector_->at(10), std::out_of_range);
}

TEST_F(SupervectorTest, Empty) {
  Supervector<double> supervector(std::vector<VectorBase<double>*>{});
  EXPECT_EQ(0, supervector.size());
}

}  // namespace
}  // namespace systems
}  // namespace drake
