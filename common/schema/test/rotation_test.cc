#include "drake/common/schema/rotation.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/common/yaml/yaml_write_archive.h"

namespace drake {
namespace schema {
namespace {

using Eigen::Vector3d;
using drake::math::RollPitchYawd;
using drake::yaml::YamlReadArchive;
using drake::yaml::YamlWriteArchive;

GTEST_TEST(RotationTest, ConstructorDefault) {
  Rotation rotation;
  EXPECT_TRUE(std::holds_alternative<Rotation::Identity>(rotation.value));
}

GTEST_TEST(RotationTest, ConstructorRotMat) {
  const Vector3d rpy_rad(0.1, 0.2, 0.3);
  const Rotation rotation{RollPitchYawd(rpy_rad).ToRotationMatrix()};
  ASSERT_TRUE(std::holds_alternative<Rotation::Rpy>(rotation.value));
  const Vector3d actual_deg = schema::GetDeterministicValue(
      std::get<Rotation::Rpy>(rotation.value).deg);
  const Vector3d expected_deg = rpy_rad * 180 / M_PI;
  EXPECT_TRUE(drake::CompareMatrices(actual_deg, expected_deg, 1e-10));
}

GTEST_TEST(RotationTest, ConstructorRpy) {
  const Vector3d rpy_rad(0.1, 0.2, 0.3);
  const Rotation rotation{RollPitchYawd(rpy_rad)};
  ASSERT_TRUE(std::holds_alternative<Rotation::Rpy>(rotation.value));
  const Vector3d actual_deg = schema::GetDeterministicValue(
      std::get<Rotation::Rpy>(rotation.value).deg);
  const Vector3d expected_deg = rpy_rad * 180 / M_PI;
  EXPECT_TRUE(drake::CompareMatrices(actual_deg, expected_deg, 1e-10));
}

GTEST_TEST(RotationTest, Rpy) {
  constexpr const char* const yaml_data = R"""(
  value: !Rpy { deg: [10., 20., 30.] }
  )""";
  Rotation rotation;
  YamlReadArchive(YAML::Load(yaml_data)).Accept(&rotation);
  ASSERT_TRUE(rotation.IsDeterministic());
  const RollPitchYawd rpy(rotation.GetDeterministicValue());
  const Vector3d actual = rpy.vector() * 180 / M_PI;
  const Vector3d expected(10.0, 20.0, 30.0);
  EXPECT_TRUE(drake::CompareMatrices(actual, expected, 1e-10));
}

GTEST_TEST(RotationTest, AngleAxis) {
  constexpr const char* const yaml_data = R"""(
  value: !AngleAxis { angle_deg: 10.0, axis: [0, 1, 0] }
  )""";
  Rotation rotation;
  YamlReadArchive(YAML::Load(yaml_data)).Accept(&rotation);
  ASSERT_TRUE(rotation.IsDeterministic());
  const Eigen::AngleAxis<double> actual =
      rotation.GetDeterministicValue().ToAngleAxis();
  EXPECT_EQ(actual.angle() * 180 / M_PI, 10.0);
  const Vector3d expected_axis(0.0, 1.0, 0.0);
  EXPECT_TRUE(drake::CompareMatrices(actual.axis(), expected_axis, 1e-10));
}

GTEST_TEST(RotationTest, Uniform) {
  constexpr const char* const yaml_data = R"""(
  value: !Uniform {}
  )""";
  Rotation rotation;
  YamlReadArchive(YAML::Load(yaml_data)).Accept(&rotation);
  EXPECT_FALSE(rotation.IsDeterministic());
  EXPECT_NO_THROW(rotation.ToSymbolic());
}

GTEST_TEST(RotationTest, RpyUniform) {
  constexpr const char* const yaml_data = R"""(
  value: !Rpy { deg: !UniformVector { min: [0, 10, 20], max: [30, 40, 50] } }
  )""";
  Rotation rotation;
  YamlReadArchive(YAML::Load(yaml_data)).Accept(&rotation);
  EXPECT_FALSE(rotation.IsDeterministic());
  // We trust stochastic_test.cc to check that stochastic.h parsed the ranges
  // correctly, and so do not verify them further here.
}

// Ensure that we can write out YAML for Identity.
GTEST_TEST(RotationTest, IdentityToYaml) {
  Rotation rotation;
  YamlWriteArchive archive;
  archive.Accept(rotation);
  EXPECT_EQ(archive.EmitString(), "root:\n  value: {}\n");
}

// Ensure that we can write out YAML for Rpy.
GTEST_TEST(RotationTest, RpyToYaml) {
  Rotation rotation;
  rotation.set_rpy_deg(Vector3d(1.0, 2.0, 3.0));
  YamlWriteArchive archive;
  archive.Accept(rotation);
  EXPECT_EQ(archive.EmitString(),
            "root:\n  value: !Rpy\n    deg: [1.0, 2.0, 3.0]\n");
}

}  // namespace
}  // namespace schema
}  // namespace drake
