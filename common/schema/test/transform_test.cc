#include "drake/common/schema/transform.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/common/yaml/yaml_io.h"
#include "drake/math/rigid_transform.h"

using drake::yaml::LoadYamlString;

namespace drake {
namespace schema {
namespace {

const char* deterministic = R"""(
base_frame: foo
translation: [1., 2., 3.]
rotation: !Rpy { deg: [10, 20, 30] }
)""";

GTEST_TEST(DeterministicTest, TransformTest) {
  const auto transform = LoadYamlString<Transform>(deterministic);

  EXPECT_EQ(*transform.base_frame, "foo");

  drake::math::RigidTransformd expected(
      drake::math::RollPitchYawd(
          Eigen::Vector3d(10., 20., 30.) * (M_PI / 180.0)),
      Eigen::Vector3d(1., 2., 3.));
  EXPECT_TRUE(drake::CompareMatrices(
      transform.GetDeterministicValue().GetAsMatrix34(),
      expected.GetAsMatrix34()));
  EXPECT_TRUE(drake::CompareMatrices(
      transform.Mean().GetAsMatrix34(),
      expected.GetAsMatrix34()));
}

const char* random = R"""(
base_frame: bar
translation: !UniformVector { min: [1., 2., 3.], max: [4., 5., 6.] }
rotation: !Uniform {}
)""";

GTEST_TEST(StochasticTest, TransformTest) {
  const auto transform = LoadYamlString<Transform>(random);

  EXPECT_EQ(*transform.base_frame, "bar");
  EXPECT_FALSE(IsDeterministic(transform.translation));
  EXPECT_TRUE(std::holds_alternative<Rotation::Uniform>(
      transform.rotation.value));
  EXPECT_TRUE(drake::CompareMatrices(
      transform.Mean().translation(),
      Eigen::Vector3d(2.5, 3.5, 4.5)));
}

const char* random_bounded = R"""(
base_frame: baz
translation: !UniformVector { min: [1., 2., 3.], max: [4., 5., 6.] }
rotation: !Rpy
  deg: !UniformVector
    min: [380, -0.25, -1.]
    max: [400,  0.25,  1.]
)""";

GTEST_TEST(StochasticSampleTest, TransformTest) {
  const auto transform = LoadYamlString<Transform>(random_bounded);
  drake::RandomGenerator generator(0);
  drake::math::RigidTransformd sampled_transform = transform.Sample(&generator);

  // The sampled transforms must lie within the randomization domain.
  // TODO(jwnimmer-tri) The correctness checks below traceable to rotation.cc's
  // implementation would be better placed in rotation_test.cc.
  const auto& translation_domain =
      std::get<schema::UniformVector<3>>(transform.translation);
  for (int i = 0; i < 3; i++) {
    EXPECT_GT(sampled_transform.translation()[i], translation_domain.min[i]);
    EXPECT_LT(sampled_transform.translation()[i], translation_domain.max[i]);
  }
  const auto& rotation_domain =
      std::get<schema::UniformVector<3>>(
          std::get<schema::Rotation::Rpy>(
              transform.rotation.value).deg);
  const Eigen::Vector3d rpy =
      drake::math::RollPitchYawd(sampled_transform.rotation())
      .vector() * (180 / M_PI);
  EXPECT_LT(rpy[0], rotation_domain.max[0] - 360);
  EXPECT_GT(rpy[0], rotation_domain.min[0] - 360);
  EXPECT_LT(rpy[1], rotation_domain.max[1]);
  EXPECT_GT(rpy[1], rotation_domain.min[1]);
  EXPECT_LT(rpy[2], rotation_domain.max[2]);
  EXPECT_GT(rpy[2], rotation_domain.min[2]);

  // A second sample will (almost certainly) differ from the first sample.
  const drake::math::RigidTransformd sampled_transform_2 =
      transform.Sample(&generator);
  EXPECT_FALSE(sampled_transform.IsExactlyEqualTo(sampled_transform_2));

  // Check the mean.
  drake::math::RigidTransformd expected_mean(
      drake::math::RollPitchYawd(
          Eigen::Vector3d(390., 0., 0.) * (M_PI / 180.0)),
      Eigen::Vector3d(2.5, 3.5, 4.5));
  EXPECT_TRUE(transform.Mean().IsExactlyEqualTo(expected_mean));
}

}  // namespace
}  // namespace schema
}  // namespace drake
