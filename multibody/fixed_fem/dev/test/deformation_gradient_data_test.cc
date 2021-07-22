#include "drake/multibody/fixed_fem/dev/deformation_gradient_data.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace fem {
namespace internal {
namespace {

/* A dummy DeformationGradientData for unit testing purpose only. It holds a
 single deformation gradient dependent data which simply doubles the
 deformation gradient. */
template <typename T, int num_locations_at_compile_time>
class DummyData : public DeformationGradientData<
                      DummyData<T, num_locations_at_compile_time>> {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(DummyData);

  DummyData() {
    double_deformation_gradient_.fill(2.0 * Matrix3<T>::Identity());
  }

  ~DummyData() = default;

  std::array<Matrix3<T>, num_locations_at_compile_time>
  double_deformation_gradient() const {
    return double_deformation_gradient_;
  }

 private:
  using Base =
      DeformationGradientData<DummyData<T, num_locations_at_compile_time>>;

  friend Base;

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

constexpr int kNumLocations = 1;
using Eigen::Matrix3d;

void VerifySizes(const DummyData<double, kNumLocations>& data) {
  ASSERT_EQ(data.deformation_gradient().size(), kNumLocations);
  ASSERT_EQ(data.double_deformation_gradient().size(), kNumLocations);
}

GTEST_TEST(DeformationGradientDataTest, DummyData) {
  DummyData<double, kNumLocations> dummy_data;
  VerifySizes(dummy_data);

  EXPECT_TRUE(CompareMatrices(dummy_data.deformation_gradient()[0],
                              Matrix3d::Identity()));
  EXPECT_TRUE(CompareMatrices(dummy_data.double_deformation_gradient()[0],
                              2.0 * Matrix3d::Identity()));

  const Matrix3d F = 2.0 * Matrix3d::Identity();
  dummy_data.UpdateData({F});

  EXPECT_TRUE(CompareMatrices(dummy_data.deformation_gradient()[0],
                              2.0 * Matrix3d::Identity()));
  EXPECT_TRUE(CompareMatrices(dummy_data.double_deformation_gradient()[0],
                              4.0 * Matrix3d::Identity()));
}

}  // namespace
}  // namespace internal
}  // namespace fem
}  // namespace multibody
}  // namespace drake
