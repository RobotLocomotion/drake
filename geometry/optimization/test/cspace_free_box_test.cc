#include "drake/geometry/optimization/cspace_free_box.h"

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"

namespace drake {
namespace geometry {
namespace optimization {

TEST_F(CIrisToyRobotTest, ComputeSBox) {
  CspaceFreeBoxTester tester(plant_, scene_graph_,
                             SeparatingPlaneOrder::kAffine);

  const Eigen::VectorXd q_position_lower = plant_->GetPositionLowerLimits();
  const Eigen::VectorXd q_position_upper = plant_->GetPositionUpperLimits();
  // First test both q_box_lower and q_box_upper within [q_position_lower,
  // q_position_upper].
  Eigen::VectorXd s_box_lower;
  Eigen::VectorXd s_box_upper;
  Eigen::VectorXd q_star;

  Eigen::VectorXd q_box_lower = 0.9 * q_position_lower + 0.1 * q_position_upper;
  Eigen::VectorXd q_box_upper = 0.1 * q_position_lower + 0.9 * q_position_upper;

  tester.ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                     &q_star);

  EXPECT_TRUE(CompareMatrices(
      s_box_lower,
      tester.cspace_free_box().rational_forward_kin().ComputeSValue(q_box_lower,
                                                                    q_star)));

  // Now test q_box_upper larger than q_position_upper.
  q_box_upper = q_position_upper +
                Eigen::VectorXd::Constant(q_position_upper.rows(), 0.1);
  tester.ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                     &q_star);
  EXPECT_TRUE(CompareMatrices(q_star, 0.5 * (q_box_lower + q_position_upper)));

  EXPECT_TRUE(CompareMatrices(
      s_box_lower,
      tester.cspace_free_box().rational_forward_kin().ComputeSValue(q_box_lower,
                                                                    q_star)));
  EXPECT_TRUE(CompareMatrices(
      s_box_upper,
      tester.cspace_free_box().rational_forward_kin().ComputeSValue(
          q_position_upper, q_star)));

  // Test q_box_lower smaller than q_position_lower.
  q_box_lower = q_position_lower -
                Eigen::VectorXd::Constant(q_position_lower.rows(), 0.1);
  q_box_upper = 0.1 * q_position_lower + 0.9 * q_position_upper;
  tester.ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                     &q_star);
  EXPECT_TRUE(CompareMatrices(q_star, 0.5 * (q_position_lower + q_box_upper)));
  EXPECT_TRUE(CompareMatrices(
      s_box_lower,
      tester.cspace_free_box().rational_forward_kin().ComputeSValue(
          q_position_lower, q_star)));
  EXPECT_TRUE(CompareMatrices(
      s_box_upper,
      tester.cspace_free_box().rational_forward_kin().ComputeSValue(q_box_upper,
                                                                    q_star)));

  // Test q_box_lower larger than q_box_upper.
  q_box_lower = 0.9 * q_position_lower + 0.1 * q_position_upper;
  q_box_upper = 0.1 * q_position_lower + 0.9 * q_position_upper;
  q_box_lower(0) = q_box_upper(0) + 0.1;
  DRAKE_EXPECT_THROWS_MESSAGE(
      tester.ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                         &q_star),
      ".* has some entries larger than q_box_upper.*");
}

TEST_F(CIrisToyRobotTest, GeneratePolynomialsToCertify) {
  CspaceFreeBoxTester tester(plant_, scene_graph_,
                             SeparatingPlaneOrder::kAffine);
  const Eigen::VectorXd q_position_lower = plant_->GetPositionLowerLimits();
  const Eigen::VectorXd q_position_upper = plant_->GetPositionUpperLimits();
  const Eigen::VectorXd q_box_lower =
      0.9 * q_position_lower + 0.1 * q_position_upper;
  const Eigen::VectorXd q_box_upper =
      0.2 * q_position_lower + 0.8 * q_position_upper;
  Eigen::VectorXd s_box_lower;
  Eigen::VectorXd s_box_upper;
  Eigen::VectorXd q_star;
  tester.ComputeSBox(q_box_lower, q_box_upper, &s_box_lower, &s_box_upper,
                     &q_star);

  CspaceFreeBoxTester::PolynomialsToCertify certify_polynomials;
  CspaceFreeBox::IgnoredCollisionPairs ignored_collision_pairs;
  ignored_collision_pairs.emplace(world_box_, body3_cylinder_);
  tester.GeneratePolynomialsToCertify(s_box_lower, s_box_upper, q_star,
                                      ignored_collision_pairs,
                                      &certify_polynomials);
  // First test s-s_box_lower and s_box_upper-s.
  const VectorX<symbolic::Variable>& s =
      tester.cspace_free_box().rational_forward_kin().s();
  const int s_size = s.rows();
  EXPECT_EQ(certify_polynomials.data.s_box_upper_minus_s.rows(), s_size);
  EXPECT_EQ(certify_polynomials.data.s_minus_s_box_lower.rows(), s_size);
  for (int i = 0; i < s_size; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual,
                 certify_polynomials.data.s_box_upper_minus_s(i),
                 symbolic::Polynomial(s_box_upper(i) - s(i)));
    EXPECT_PRED2(symbolic::test::PolyEqual,
                 certify_polynomials.data.s_minus_s_box_lower(i),
                 symbolic::Polynomial(s(i) - s_box_lower(i)));
  }
  EXPECT_EQ(certify_polynomials.data.plane_geometries.size(),
            tester.cspace_free_box().separating_planes().size() -
                ignored_collision_pairs.size());
  for (const auto& plane_geometries :
       certify_polynomials.data.plane_geometries) {
    const auto& plane = tester.cspace_free_box()
                            .separating_planes()[plane_geometries.plane_index];
    if (plane.positive_side_geometry->type() == CIrisGeometryType::kPolytope &&
        plane.negative_side_geometry->type() == CIrisGeometryType::kPolytope) {
      EXPECT_EQ(plane_geometries.positive_side_rationals.size(),
                plane.positive_side_geometry->num_rationals());
      EXPECT_EQ(plane_geometries.negative_side_rationals.size(),
                plane.negative_side_geometry->num_rationals());
    } else if (plane.positive_side_geometry->type() ==
                   CIrisGeometryType::kPolytope &&
               plane.negative_side_geometry->type() !=
                   CIrisGeometryType::kPolytope) {
      EXPECT_EQ(plane_geometries.positive_side_rationals.size(),
                plane.positive_side_geometry->num_rationals());
      EXPECT_EQ(plane_geometries.negative_side_rationals.size(),
                plane.negative_side_geometry->num_rationals() - 1);
    } else if (plane.positive_side_geometry->type() !=
                   CIrisGeometryType::kPolytope &&
               plane.negative_side_geometry->type() ==
                   CIrisGeometryType::kPolytope) {
      EXPECT_EQ(plane_geometries.positive_side_rationals.size(),
                plane.positive_side_geometry->num_rationals() - 1);
      EXPECT_EQ(plane_geometries.negative_side_rationals.size(),
                plane.negative_side_geometry->num_rationals());
    } else {
      EXPECT_EQ(plane_geometries.positive_side_rationals.size(),
                plane.positive_side_geometry->num_rationals());
      EXPECT_EQ(plane_geometries.negative_side_rationals.size(),
                plane.negative_side_geometry->num_rationals() - 1);
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
