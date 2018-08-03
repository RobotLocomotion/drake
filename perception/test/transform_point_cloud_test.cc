#include "drake/perception/transform_point_cloud.h"

#include <cmath>
#include <random>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace perception {
namespace {

class TransformPointCloudTest : public ::testing::Test {
 public:
  static Matrix3X<float> GenerateBoundedSample(const Vector3<float>& min,
                                               const Vector3<float>& max,
                                               int num_cols) {
    Matrix3X<float> return_matrix = Matrix3X<float>::Zero(3, num_cols);
    const Vector3<float> increment = (max - min) / static_cast<float>(num_cols);

    for (int i = 0; i < num_cols; ++i) {
      return_matrix.col(i) = increment * static_cast<float>(i);
    }

    return return_matrix;
  }

 protected:
  void SetUp() override {
    transformer_ = std::make_unique<TransformPointCloud>();
    context_ = transformer_->CreateDefaultContext();
    output_ = transformer_->point_cloud_output_port().Allocate();
    point_cloud_input_ =
        systems::AbstractValue::Make<PointCloud>(PointCloud(0));
    rigid_transform_input_ =
        systems::AbstractValue::Make<math::RigidTransform<float>>(
            math::RigidTransform<float>());
  }

  std::unique_ptr<TransformPointCloud> transformer_;
  std::unique_ptr<systems::Context<double>> context_;
  std::unique_ptr<systems::AbstractValue> output_;
  std::unique_ptr<systems::AbstractValue> point_cloud_input_;
  std::unique_ptr<systems::AbstractValue> rigid_transform_input_;
};

// Verifies that the system applies the transform correctly to the point cloud.
TEST_F(TransformPointCloudTest, ApplyTransformTest) {
  const Vector3<float> kMin(-10.0, -20.0, -30.0);
  const Vector3<float> kMax(10.0, 20.0, 30.0);
  const int kNumPoints = 5;
  const Vector3<float> kRpy(M_PI_4, -M_PI_2, 0.543);
  const math::RollPitchYaw<float> kRollPitchYaw(kRpy);
  const math::RotationMatrix<float> kR(kRollPitchYaw);
  const Vector3<float> kP(-0.3, 5.4, -2.7);

  MatrixX<float> test_data =
      TransformPointCloudTest::GenerateBoundedSample(kMin, kMax, kNumPoints);

  PointCloud cloud(kNumPoints);
  cloud.mutable_xyzs() = test_data;

  math::RigidTransform<float> transform(kR, kP);

  context_->FixInputPort(0, systems::AbstractValue::Make<PointCloud>(cloud));
  context_->FixInputPort(
      1, systems::AbstractValue::Make<math::RigidTransform<float>>(transform));

  transformer_->point_cloud_output_port().Calc(*context_, output_.get());

  auto output_cloud = output_->GetValueOrThrow<PointCloud>();

  Matrix4X<float> test_data_homogeneous(4, kNumPoints);
  test_data_homogeneous.block(0, 0, 3, kNumPoints) = test_data;
  test_data_homogeneous.row(3) = VectorX<float>::Ones(kNumPoints);
  Matrix4X<float> expected_output =
      transform.GetAsMatrix4() * test_data_homogeneous;

  // The tolerance used here has this value because the point cloud and the
  // rigid transform both use `float` as the numerical representation.
  EXPECT_TRUE(CompareMatrices(output_cloud.xyzs(),
                              expected_output.block(0, 0, 3, kNumPoints),
                              10.0f * std::numeric_limits<float>::epsilon()));
}

}  // namespace
}  // namespace perception
}  // namespace drake
