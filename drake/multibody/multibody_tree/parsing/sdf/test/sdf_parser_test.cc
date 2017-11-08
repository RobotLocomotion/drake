#include "drake/multibody/multibody_tree/parsing/sdf/sdf_parser.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/find_resource.h"

namespace drake {
namespace multibody {
namespace multibody_tree {
namespace parsing {
namespace {

const double kEpsilon = std::numeric_limits<double>::epsilon();

#include <iostream>
#define PRINT_VAR(a) std::cout << #a": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a":\n" << a << std::endl;

static const char* const kTestSdfPath =
    "drake/multibody/multibody_tree/parsing/sdf/models/double_pendulum.sdf";

GTEST_TEST(SDFParserTest, ParsingTest) {
  // Parse tree from test SDF.
  const std::string sdf_path = FindResourceOrThrow(kTestSdfPath);
  const std::string kModelName = "double_pendulum_with_base";

  PRINT_VAR(sdf_path);

  SDFParser parser;
  auto sdf_spec = parser.ParseSDFModelFromFile(sdf_path);

  PRINT_VAR(sdf_spec->version());
  PRINT_VAR(sdf_spec->get_num_models());
  const SDFModel& model = sdf_spec->GetModelByName(kModelName);
  const int model_id = sdf_spec->GetModelIdByName("double_pendulum_with_base");
  PRINT_VAR(model.name());

  EXPECT_EQ(sdf_spec->version(), "1.6");
  EXPECT_TRUE(sdf_spec->HasModel(kModelName));
  EXPECT_EQ(model_id, 0);

  PRINT_VAR(model.get_num_links());
  EXPECT_EQ(model.get_num_links(), 3);

  const std::vector<SDFLink>& links = model.get_links();

  const auto& is_link_in_vector = [](
      const std::vector<SDFLink>& v, const std::string& name) {
    return
        std::find_if(v.begin(), v.end(), [&name](const SDFLink& link) {
          return link.name() == name;
        }) != v.end();
  };

  EXPECT_TRUE(is_link_in_vector(links, "lower_link"));
  EXPECT_TRUE(is_link_in_vector(links, "base"));
  EXPECT_TRUE(is_link_in_vector(links, "upper_link"));

  const SDFLink& lower_link = model.GetLinkByName("lower_link");
  const SDFLink& upper_link = model.GetLinkByName("upper_link");
  const Isometry3<double> X_DL = lower_link.get_pose_in_model();
  const Isometry3<double> X_DU = upper_link.get_pose_in_model();

  // Expected values of the links's poses in the model frame D.
  const Isometry3<double> X_DL_expected =
      Translation3<double>(0.25, 1.0, 2.1) *
      AngleAxis<double>(-2.0, Vector3<double>::UnitX());

  const Isometry3<double> X_DU_expected =
      Translation3<double>(0.0, 0.0, 2.1) *
      AngleAxis<double>(-1.5708, Vector3<double>::UnitX());

  EXPECT_TRUE(X_DL.isApprox(X_DL_expected, kEpsilon));
  EXPECT_TRUE(X_DU.isApprox(X_DU_expected, kEpsilon));

  EXPECT_NEAR(lower_link.mass(), 30.0, kEpsilon);
  EXPECT_NEAR(upper_link.mass(), 30.0, kEpsilon);

  // Verify the value of the <inertial> frame poses in their respective link
  // frames.
  const Isometry3<double>& X_UI = upper_link.get_inertial_frame_pose();
  const Isometry3<double>& X_LI = lower_link.get_inertial_frame_pose();

  const Isometry3<double> X_UI_expected(Translation3<double>(0.0, 0.0, 0.5));
  const Isometry3<double> X_LI_expected(Translation3<double>(0.0, 0.0, 0.5));

  EXPECT_TRUE(X_UI.isApprox(X_UI_expected, kEpsilon));
  EXPECT_TRUE(X_LI.isApprox(X_LI_expected, kEpsilon));
}

}  // namespace
}  // namespace parsing
}  // namespace multibody_tree
}  // namespace multibody
}  // namespace drake
