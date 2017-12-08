#include "drake/multibody/parsing/sdf_parser.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace drake {
namespace multibody {
namespace parsing {
namespace {

GTEST_TEST(SdfParserTest, ParsingTest) {
  // This tolerance is based off how many precision digits are specified in the
  // SDF file used for this unit test.
  const double kTolerance = 1.0e-10;

  static const char* const kTestSdfPath =
      "drake/multibody/parsing/test/models/double_pendulum.sdf";

  // Parse tree from test SDF.
  const std::string sdf_path = FindResourceOrThrow(kTestSdfPath);
  const std::string kModelName = "double_pendulum_with_base";

  auto sdf_spec = ParseSdfModelFromFile(sdf_path);
  EXPECT_EQ(sdf_spec->version(), "1.6");

  // The method above parses a single model.
  EXPECT_EQ(sdf_spec->get_num_models(), 1);
  EXPECT_TRUE(sdf_spec->HasModel(kModelName));
  const SdfModel& model = sdf_spec->GetModelByName(kModelName);

  EXPECT_EQ(model.name(), kModelName);

  // Verify links:
  EXPECT_EQ(model.get_num_links(), 3);

  const std::vector<SdfLink>& links = model.get_links();

  const auto is_link_in_vector = [](
      const std::vector<SdfLink>& v, const std::string& name) {
    return
        std::find_if(v.begin(), v.end(), [&name](const SdfLink& link) {
          return link.name() == name;
        }) != v.end();
  };

  EXPECT_TRUE(is_link_in_vector(links, "lower_link"));
  EXPECT_TRUE(is_link_in_vector(links, "base"));
  EXPECT_TRUE(is_link_in_vector(links, "upper_link"));

  const SdfLink& lower_link = model.GetLinkByName("lower_link");
  const SdfLink& upper_link = model.GetLinkByName("upper_link");
  const Isometry3<double> X_ML = model.GetPoseInModelFrame(lower_link.name());
  const Isometry3<double> X_MU = model.GetPoseInModelFrame(upper_link.name());

  // Expected values of the links's poses in the model frame D.
  const Isometry3<double> X_ML_expected =
      Translation3<double>(0.25, 1.0, 2.1) *
      AngleAxis<double>(-2.0, Vector3<double>::UnitX());

  const Isometry3<double> X_MU_expected =
      Translation3<double>(0.0, 0.0, 2.1) *
      AngleAxis<double>(-1.5708, Vector3<double>::UnitX());

  EXPECT_TRUE(X_ML.isApprox(X_ML_expected, kTolerance));
  EXPECT_TRUE(X_MU.isApprox(X_MU_expected, kTolerance));

  EXPECT_NEAR(lower_link.mass(), 30.0, kTolerance);
  EXPECT_NEAR(upper_link.mass(), 30.0, kTolerance);

  // Verify the value of the <inertial> frame poses in their respective link
  // frames.
  const Isometry3<double>& X_UIcm = upper_link.get_inertial_frame_pose();
  const Isometry3<double>& X_LIcm = lower_link.get_inertial_frame_pose();

  const Isometry3<double> X_UIcm_expected(Translation3<double>(0.0, 0.0, 0.5));
  const Isometry3<double> X_LIcm_expected(Translation3<double>(0.0, 0.0, 0.5));

  EXPECT_TRUE(X_UIcm.isApprox(X_UIcm_expected, kTolerance));
  EXPECT_TRUE(X_LIcm.isApprox(X_LIcm_expected, kTolerance));

  // Verify the value of the inertia matrix for each link.
  const Matrix3<double>& I_Icm = upper_link.get_inertia_matrix();
  const Matrix3<double> I_Icm_expected = (Matrix3<double>()
      << 1.0, 0.1, 0.2,
         0.1, 2.0, 0.3,
         0.2, 0.3, 3.0).finished();
  EXPECT_TRUE(CompareMatrices(I_Icm, I_Icm_expected, kTolerance,
                              MatrixCompareType::relative));

  // Verify joints:
  EXPECT_EQ(model.get_num_joints(), 2);
  const std::vector<SdfJoint>& joints = model.get_joints();

  const auto is_joint_in_vector = [](
      const std::vector<SdfJoint>& v, const std::string& name) {
    return
        std::find_if(v.begin(), v.end(), [&name](const SdfJoint& joint) {
          return joint.name() == name;
        }) != v.end();
  };
  EXPECT_TRUE(is_joint_in_vector(joints, "upper_joint"));
  EXPECT_TRUE(is_joint_in_vector(joints, "lower_joint"));

  const SdfJoint& lower_joint = model.GetJointByName("lower_joint");
  const SdfJoint& upper_joint = model.GetJointByName("upper_joint");

  // TODO(amcastro-tri): write more unit tests per PR #4615 that properly
  // verify the poses of joint and link frames.
  EXPECT_EQ(upper_joint.joint_type(), "revolute");
  EXPECT_EQ(upper_joint.parent_link(), "base");
  EXPECT_EQ(upper_joint.child_link(), "upper_link");
  EXPECT_TRUE(upper_joint.get_axis().isApprox(
      Vector3<double>::UnitX(), kTolerance));

  EXPECT_EQ(lower_joint.joint_type(), "revolute");
  EXPECT_EQ(lower_joint.parent_link(), "upper_link");
  EXPECT_EQ(lower_joint.child_link(), "lower_link");
  EXPECT_TRUE(lower_joint.get_axis().isApprox(
      Vector3<double>::UnitX(), kTolerance));

  // To create a model out of the SDF specs we'll need to get the pose of a
  // joint frame J in its child link frame. Here we test that:
  const Isometry3<double> X_UJu =
      model.GetPose(upper_link.name(), upper_joint.name());
  const Matrix3<double> R_UJu_expected(
      Eigen::AngleAxisd(M_PI / 2.0, Vector3<double>::UnitY()));
  const Vector3<double> p_UJu_expected(-0.025, 0.0, 0.0);
  EXPECT_TRUE(X_UJu.translation().isApprox(p_UJu_expected, kTolerance));
  EXPECT_TRUE(X_UJu.linear().isApprox(R_UJu_expected, kTolerance));

  const Isometry3<double> X_LJl =
      model.GetPose(lower_link.name(), lower_joint.name());
  EXPECT_TRUE(X_LJl.isApprox(Isometry3<double>::Identity(), kTolerance));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody
}  // namespace drake
