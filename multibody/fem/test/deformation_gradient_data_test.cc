#include "drake/multibody/fem/deformation_gradient_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

constexpr int kNumLocations = 1;
using Eigen::Matrix3d;

/* A dummy DeformationGradientData for testing the behaviors of
 DeformationGradientData::UpdateData(). It holds a single deformation
 gradient dependent data which simply doubles the deformation gradient. */
template <typename T, int num_locations_at_compile_time>
class DummyData : public DeformationGradientData<
                      DummyData<T, num_locations_at_compile_time>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyData);

  DummyData() {
    /* Initialize to a consistent state where `double_deformation_gradient_` is
     twice the `deformation_gradient()`, but the unit test doesn't actually
     require it. */
    UpdateFromDeformationGradient();
  }

  std::array<Matrix3<T>, num_locations_at_compile_time>
  double_deformation_gradient() const {
    return double_deformation_gradient_;
  }

 private:
  friend DeformationGradientData<DummyData<T, num_locations_at_compile_time>>;

  /* Shadows DeformationGradientData::UpdateFromDeformationGradient() as
   required by the CRTP base class. */
  void UpdateFromDeformationGradient() {
    const std::array<Matrix3<T>, num_locations_at_compile_time>& F =
        this->deformation_gradient();
    for (int i = 0; i < num_locations_at_compile_time; ++i)
      double_deformation_gradient_[i] = 2.0 * F[i];
  }

  std::array<Matrix3<T>, num_locations_at_compile_time>
      double_deformation_gradient_;
};

void VerifySizes(const DummyData<double, kNumLocations>& data) {
  ASSERT_EQ(data.deformation_gradient().size(), kNumLocations);
  ASSERT_EQ(data.double_deformation_gradient().size(), kNumLocations);
}

/* Verifies that `DeformationGradientData::UpdateData()` does what it promises
to do. In particular, it
1. updates the deformation data to be the input deformation gradient, and
2. invokes UpdateFromDeformationGradient() in the derived class *after* the
deformation gradient is up to date. */
GTEST_TEST(DeformationGradientDataTest, UpdateData) {
  DummyData<double, kNumLocations> dummy_data;
  /* Verify that the `dummy_data` instance has the expected size so that the
   indexing in the following tests will not go out of bound. */
  VerifySizes(dummy_data);

  ASSERT_TRUE(CompareMatrices(dummy_data.deformation_gradient()[0],
                              Matrix3d::Identity()));
  ASSERT_TRUE(CompareMatrices(dummy_data.double_deformation_gradient()[0],
                              2.0 * Matrix3d::Identity()));

  /* Arbitrary deformation gradient. */
  Matrix3d F;
  // clang-format off
  F << 4, 9, 2,
       3, 5, 7,
       8, 1, 6;
  // clang-format on
  dummy_data.UpdateData({F});

  EXPECT_TRUE(CompareMatrices(dummy_data.deformation_gradient()[0], F));
  EXPECT_TRUE(
      CompareMatrices(dummy_data.double_deformation_gradient()[0], 2.0 * F));
}

}  // namespace

/* An invalid DeformationGradientData that is missing the
 UpdateFromDeformationGradient() method. This class is used to test that an
 exception is thrown in the case where the derived class of
 DeformationGradientData doesn't shadow the UpdateFromDeformationGradient()
 method. */
template <typename T, int num_locations_at_compile_time>
class InvalidDummyData
    : public DeformationGradientData<
          InvalidDummyData<T, num_locations_at_compile_time>> {};

namespace {

/* Verifies that if a derived class of `DeformationGradientData` doesn't shadow
 the `UpdateFromDeformationGradient()` method, an exception is thrown. */
GTEST_TEST(DeformationGradientDataTest,
           UnshadowedUpdateFromDeformationGradient) {
  InvalidDummyData<double, kNumLocations> invalid_dummy_data;
  const Matrix3d F = 2.8 * Matrix3d::Identity();
  DRAKE_EXPECT_THROWS_MESSAGE(
      invalid_dummy_data.UpdateData({F}),
      fmt::format("The derived class {} must provide a shadow definition of "
                  "UpdateFromDeformationGradient.. to be correct.",
                  NiceTypeName::Get(invalid_dummy_data)));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
