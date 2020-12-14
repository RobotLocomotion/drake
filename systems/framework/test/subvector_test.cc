#include "drake/systems/framework/subvector.h"

#include <memory>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "drake/common/autodiff.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/systems/framework/basic_vector.h"

namespace drake {
namespace systems {
namespace {

const int kSubVectorLength = 2;

class SubvectorTest : public ::testing::Test {
 protected:
  void SetUp() override { vector_ = BasicVector<double>::Make({1, 2, 3, 4}); }

  std::unique_ptr<VectorBase<double>> vector_;
};

TEST_F(SubvectorTest, NullptrVector) {
  EXPECT_THROW(Subvector<double> subvec(nullptr, 0, 0), std::exception);
}

TEST_F(SubvectorTest, OutOfBoundsSubvector) {
  EXPECT_THROW(Subvector<double>(vector_.get(), 1, 4), std::exception);
}

TEST_F(SubvectorTest, Access) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_EQ(2, subvec.GetAtIndex(0));
  EXPECT_EQ(3, subvec.GetAtIndex(1));
  const auto& const_subvec = subvec;
  EXPECT_EQ(2, const_subvec.GetAtIndex(0));
  EXPECT_EQ(3, const_subvec.GetAtIndex(1));
}

// Tests that access out of bounds throws an exception.
TEST_F(SubvectorTest, OutOfRange) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec.GetAtIndex(-1), std::exception);
  EXPECT_THROW(subvec.GetAtIndex(kSubVectorLength), std::exception);
  EXPECT_THROW(subvec.SetAtIndex(-1, 0.0), std::exception);
  EXPECT_THROW(subvec.SetAtIndex(kSubVectorLength, 0.0), std::exception);
}

TEST_F(SubvectorTest, Copy) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  Eigen::Vector2d expected;
  expected << 2, 3;
  EXPECT_EQ(expected, subvec.CopyToVector());

  Eigen::Vector2d pre_sized_good;
  subvec.CopyToPreSizedVector(&pre_sized_good);
  EXPECT_EQ(expected, pre_sized_good);

  Eigen::Vector3d pre_sized_bad;
  EXPECT_THROW(subvec.CopyToPreSizedVector(&pre_sized_bad),
      std::exception);
}

// Tests that writes to the subvector pass through to the sliced vector.
TEST_F(SubvectorTest, Mutation) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  VectorX<double> next_value(kSubVectorLength);
  next_value << 5, 6;
  subvec.SetFromVector(next_value);
  EXPECT_EQ(5, subvec.GetAtIndex(0));
  EXPECT_EQ(6, subvec.GetAtIndex(1));

  subvec.SetAtIndex(1, 42);
  EXPECT_EQ(1, vector_->GetAtIndex(0));
  EXPECT_EQ(5, vector_->GetAtIndex(1));
  EXPECT_EQ(42, vector_->GetAtIndex(2));
  EXPECT_EQ(4, vector_->GetAtIndex(3));

  EXPECT_THROW(subvec.SetFromVector(Eigen::Vector3d::Zero()), std::exception);
}

// Tests that a subvector can be SetFrom another VectorBase.
TEST_F(SubvectorTest, SetFrom) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  auto next_value = BasicVector<double>::Make({7, 8});
  subvec.SetFrom(*next_value);
  EXPECT_EQ(7, subvec.GetAtIndex(0));
  EXPECT_EQ(8, subvec.GetAtIndex(1));

  auto bad_value = BasicVector<double>::Make({1, 2, 3});
  EXPECT_THROW(subvec.SetFrom(*bad_value), std::exception);
}

// Tests that the Subvector can be addressed as an array.
TEST_F(SubvectorTest, ArrayOperator) {
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  subvec[0] = 42;
  EXPECT_EQ(42, vector_->GetAtIndex(1));

  const auto& const_subvec = subvec;
  EXPECT_EQ(3, const_subvec[1]);
}

// Tests that a VectorBase can be added to a Subvector.
TEST_F(SubvectorTest, PlusEq) {
  const BasicVector<double> addend{7, 8};

  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  subvec += addend;

  EXPECT_EQ(1, vector_->GetAtIndex(0));
  EXPECT_EQ(9, vector_->GetAtIndex(1));
  EXPECT_EQ(11, vector_->GetAtIndex(2));
  EXPECT_EQ(4, vector_->GetAtIndex(3));
}

// Tests that a Subvector can be added to an Eigen vector.
TEST_F(SubvectorTest, ScaleAndAddToVector) {
  VectorX<double> target(2);
  target << 100, 1000;

  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  subvec.ScaleAndAddToVector(1, &target);

  Eigen::Vector2d expected;
  expected << 102, 1003;
  EXPECT_EQ(expected, target);
}

// TODO(david-german-tri): Once GMock is available in the Drake build, add a
// test case demonstrating that the += operator on Subvector calls
// ScaleAndAddToVector on the addend.

TEST_F(SubvectorTest, PlusEqInvalidSize) {
  BasicVector<double> addend(1);
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec += addend, std::exception);
}

TEST_F(SubvectorTest, AddToVectorInvalidSize) {
  VectorX<double> target(3);
  Subvector<double> subvec(vector_.get(), 1, kSubVectorLength);
  EXPECT_THROW(subvec.ScaleAndAddToVector(1, &target), std::exception);
}

// Tests SetZero functionality in VectorBase.
TEST_F(SubvectorTest, SetZero) {
  Subvector<double> subvec(vector_.get(), 0, kSubVectorLength);
  subvec.SetZero();
  for (int i = 0; i < subvec.size(); i++) {
    EXPECT_EQ(subvec.GetAtIndex(i), 0);
  }
}

// Tests all += * operations for VectorBase.
TEST_F(SubvectorTest, PlusEqScaled) {
  Subvector<double> orig_vec(vector_.get(), 0, kSubVectorLength);
  BasicVector<double> vec1{1, 2};
  BasicVector<double> vec2{3, 5};
  BasicVector<double> vec3{7, 11};
  BasicVector<double> vec4{13, 17};
  BasicVector<double> vec5{19, 23};
  VectorBase<double>& v1 = vec1;
  VectorBase<double>& v2 = vec2;
  VectorBase<double>& v3 = vec3;
  VectorBase<double>& v4 = vec4;
  VectorBase<double>& v5 = vec5;

  orig_vec.SetZero();
  orig_vec.PlusEqScaled(2, v1);
  EXPECT_EQ(orig_vec.GetAtIndex(0), 2);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 4);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 11);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 19);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 46);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 74);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}, {7, v4}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 137);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 193);

  orig_vec.SetZero();
  orig_vec.PlusEqScaled({{2, v1}, {3, v2}, {5, v3}, {7, v4}, {11, v5}});
  EXPECT_EQ(orig_vec.GetAtIndex(0), 346);
  EXPECT_EQ(orig_vec.GetAtIndex(1), 446);
}

}  // namespace
}  // namespace systems
}  // namespace drake
