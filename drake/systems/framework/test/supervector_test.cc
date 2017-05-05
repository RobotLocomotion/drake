#include "drake/systems/framework/supervector.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/vector_base.h"
#include "drake/systems/framework/subvector.h"

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
  ASSERT_EQ(kLength, supervector_->size());
  for (int i = 0; i < kLength; ++i) {
    EXPECT_EQ(i, supervector_->GetAtIndex(i));
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
  EXPECT_EQ(8, (*supervector_)[8]);
}

TEST_F(SupervectorTest, OutOfRange) {
  EXPECT_THROW(supervector_->GetAtIndex(-1), std::out_of_range);
  EXPECT_THROW(supervector_->GetAtIndex(10), std::out_of_range);
}

TEST_F(SupervectorTest, Empty) {
  Supervector<double> supervector(std::vector<VectorBase<double>*>{});
  EXPECT_EQ(0, supervector.size());
}

// A Supervector composed of several chunks is, in general, not contiguous in
// memory.
TEST_F(SupervectorTest, IsNotContiguous) {
  EXPECT_FALSE(supervector_->is_contiguous());
  EXPECT_FALSE(supervector_->IsContiguous());
}

// A Supervector could be contiguous in memory if, for instance, only composed
// of a single Subvector.
TEST_F(SupervectorTest, IsContiguous) {
  auto contiguous_supervector =
      std::make_unique<Supervector<double>>(
          std::vector<VectorBase<double>*>{vec1_.get()});
  EXPECT_TRUE(contiguous_supervector->is_contiguous());
  EXPECT_TRUE(contiguous_supervector->IsContiguous());
  for (int i = 0; i < contiguous_supervector->size(); ++i) {
    EXPECT_EQ(vec1_->GetAtIndex(i), contiguous_supervector->GetAtIndex(i));
  }
}

GTEST_TEST(SupervectorIsContiguous, PatologicalCase) {
  auto long_basic_vector =
      BasicVector<double>::Make({0, 1, 2, 3, 4, 5, 6, 7, 8});
  Subvector<double> subvec1(long_basic_vector.get(),
                            0 /* first element */, 4 /* size */);
  Subvector<double> subvec2(long_basic_vector.get(),
                            4 /* first element */, 5 /* size */);
  EXPECT_TRUE(subvec1.is_contiguous());
  EXPECT_TRUE(subvec2.is_contiguous());
  EXPECT_TRUE(subvec1.IsContiguous());
  EXPECT_TRUE(subvec2.IsContiguous());

  // The two Subvector objects actually map to an entire contiguous chunk of
  // memory.
  auto contiguous_supervector =
      std::make_unique<Supervector<double>>(
          std::vector<VectorBase<double>*>{&subvec1, &subvec2});
  EXPECT_TRUE(contiguous_supervector->is_contiguous());
  // IsContiguous FAILS IN THIS CASE!
  EXPECT_FALSE(contiguous_supervector->IsContiguous());

  // A slice into contiguous_supervector. The entire slice fits in a contiguous
  // chunk of memory. However, since it is created from two Subvector objects,
  // we lost the information about these two slices being adjacent to each
  // other.
  Subvector<double> middle_section(
      contiguous_supervector.get(), 2 /* first element */, 4 /* size */);
  EXPECT_TRUE(middle_section.is_contiguous());
  // IsContiguous FAILS IN THIS CASE!
  EXPECT_FALSE(middle_section.IsContiguous());
}

}  // namespace
}  // namespace systems
}  // namespace drake
