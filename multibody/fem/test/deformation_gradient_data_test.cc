#include "drake/multibody/fem/deformation_gradient_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

using Eigen::Matrix3d;

/* A dummy DeformationGradientData for testing the behaviors of
 DeformationGradientData::UpdateData(). It holds a single deformation
 gradient dependent data which simply doubles the deformation gradient. */
template <typename T>
class DummyData : public DeformationGradientData<DummyData<T>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyData);

  DummyData() {
    /* Initialize to a consistent state where `double_deformation_gradient_` is
     twice the `deformation_gradient()`, but the unit test doesn't actually
     require it. */
    UpdateFromDeformationGradient();
  }

  const Matrix3<T>& double_deformation_gradient() const {
    return double_deformation_gradient_;
  }

 private:
  friend DeformationGradientData<DummyData<T>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient() {
    double_deformation_gradient_ = 2.0 * this->deformation_gradient();
  }

  Matrix3<T> double_deformation_gradient_;
};

/* Verifies that `DeformationGradientData::UpdateData()` does what it promises
to do. In particular, it
1. updates the deformation data to be the input deformation gradient, and
2. invokes UpdateFromDeformationGradient() in the derived class *after* the
deformation gradient is up to date. */
GTEST_TEST(DeformationGradientDataTest, UpdateData) {
  DummyData<double> dummy_data;
  ASSERT_TRUE(
      CompareMatrices(dummy_data.deformation_gradient(), Matrix3d::Identity()));
  ASSERT_TRUE(CompareMatrices(dummy_data.double_deformation_gradient(),
                              2.0 * Matrix3d::Identity()));

  /* Arbitrary deformation gradient. */
  Matrix3d F;
  // clang-format off
  F << 4, 9, 2,
       3, 5, 7,
       8, 1, 6;
  // clang-format on
  const auto F0 = 1.23 * F;
  dummy_data.UpdateData(F, F0);

  EXPECT_TRUE(CompareMatrices(dummy_data.deformation_gradient(), F));
  EXPECT_TRUE(
      CompareMatrices(dummy_data.previous_step_deformation_gradient(), F0));
  EXPECT_TRUE(
      CompareMatrices(dummy_data.double_deformation_gradient(), 2.0 * F));
}

}  // namespace

/* An invalid DeformationGradientData that is missing the
 UpdateFromDeformationGradient() method. This class is used to test that an
 exception is thrown in the case where the derived class of
 DeformationGradientData doesn't shadow the UpdateFromDeformationGradient()
 method. */
template <typename T>
class InvalidDummyData : public DeformationGradientData<InvalidDummyData<T>> {};

namespace {

/* Verifies that if a derived class of `DeformationGradientData` doesn't shadow
 the `UpdateFromDeformationGradient()` method, an exception is thrown. */
GTEST_TEST(DeformationGradientDataTest,
           UnshadowedUpdateFromDeformationGradient) {
  InvalidDummyData<double> invalid_dummy_data;
  const Matrix3d F = 2.8 * Matrix3d::Identity();
  DRAKE_EXPECT_THROWS_MESSAGE(
      invalid_dummy_data.UpdateData(F, F),
      fmt::format("The derived class {} must provide a shadow definition of "
                  "UpdateFromDeformationGradient.. to be correct.",
                  NiceTypeName::Get(invalid_dummy_data)));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
