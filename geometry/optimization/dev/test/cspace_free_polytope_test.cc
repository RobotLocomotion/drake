#include "drake/geometry/optimization/dev/cspace_free_polytope.h"

#include <limits>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/collision_filter_declaration.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/optimization/dev/test/c_iris_test_utilities.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/multibody/rational/rational_forward_kinematics.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {
const double kInf = std::numeric_limits<double>::infinity();

TEST_F(CIrisToyRobotTest, GetCollisionGeometries) {
  const auto link_geometries = GetCollisionGeometries(*plant_, *scene_graph_);
  // Each link has some geometries.
  EXPECT_EQ(link_geometries.size(), plant_->num_bodies());

  auto check_link_geometries =
      [&link_geometries](const multibody::BodyIndex body,
                         const std::unordered_set<geometry::GeometryId>&
                             geometry_ids_expected) {
        auto it = link_geometries.find(body);
        std::unordered_set<geometry::GeometryId> geometry_ids;
        for (const auto& geometry : it->second) {
          EXPECT_EQ(geometry->body_index(), body);
          geometry_ids.emplace(geometry->id());
        }
        EXPECT_EQ(geometry_ids.size(), geometry_ids_expected.size());
        for (const auto id : geometry_ids) {
          EXPECT_GT(geometry_ids_expected.count(id), 0);
        }
      };

  check_link_geometries(plant_->world_body().index(),
                        {world_box_, world_cylinder_});
  check_link_geometries(body_indices_[0], {body0_box_, body0_sphere_});
  check_link_geometries(body_indices_[1], {body1_convex_, body1_capsule_});
  check_link_geometries(body_indices_[2], {body2_sphere_, body2_capsule_});
  check_link_geometries(body_indices_[3], {body3_box_, body3_cylinder_});
}

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeConstructor) {
  // Test CspaceFreePolytope constructor.
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  const CspaceFreePolytope& dut = tester.cspace_free_polytope();
  int num_planes_expected = 0;

  const auto link_geometries = GetCollisionGeometries(*plant_, *scene_graph_);
  // Count the expected number of planes by hand.
  num_planes_expected +=
      link_geometries.at(plant_->world_body().index()).size() *
      // Don't include world_body to body0 as there is only a weld joint between
      // them.
      (link_geometries.at(body_indices_[1]).size() +
       link_geometries.at(body_indices_[2]).size() +
       link_geometries.at(body_indices_[3]).size());
  num_planes_expected += link_geometries.at(body_indices_[0]).size() *
                         link_geometries.at(body_indices_[2]).size();
  num_planes_expected += link_geometries.at(body_indices_[1]).size() *
                         link_geometries.at(body_indices_[3]).size();
  num_planes_expected += link_geometries.at(body_indices_[2]).size() *
                         link_geometries.at(body_indices_[3]).size();
  EXPECT_EQ(dut.separating_planes().size(), num_planes_expected);

  const symbolic::Variables s_set{dut.rational_forward_kin().s()};

  for (const auto& [geometry_pair, plane_index] :
       dut.map_geometries_to_separating_planes()) {
    // check plane
    const auto& plane = dut.separating_planes()[plane_index];
    if (plane.positive_side_geometry->id() <
        plane.negative_side_geometry->id()) {
      EXPECT_EQ(geometry_pair.first(), plane.positive_side_geometry->id());
      EXPECT_EQ(geometry_pair.second(), plane.negative_side_geometry->id());
    } else {
      EXPECT_EQ(geometry_pair.first(), plane.negative_side_geometry->id());
      EXPECT_EQ(geometry_pair.second(), plane.positive_side_geometry->id());
    }
    // Check the expressed body.
    EXPECT_EQ(plane.expressed_body,
              multibody::internal::FindBodyInTheMiddleOfChain(
                  *plant_, plane.positive_side_geometry->body_index(),
                  plane.negative_side_geometry->body_index()));
    for (int i = 0; i < 3; ++i) {
      EXPECT_EQ(plane.a(i).TotalDegree(), 1);
      EXPECT_EQ(plane.a(i).indeterminates(), s_set);
    }
    EXPECT_EQ(plane.b.TotalDegree(), 1);
    EXPECT_EQ(plane.b.indeterminates(), s_set);
  }
}

TEST_F(CIrisToyRobotTest, CspaceFreePolytopeGenerateRationals) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  EXPECT_EQ(tester.plane_geometries().size(),
            tester.cspace_free_polytope().separating_planes().size());
  for (const auto& plane_geometries : tester.plane_geometries()) {
    const auto& plane = tester.cspace_free_polytope()
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

TEST_F(CIrisToyRobotTest, FindRedundantInequalities) {
  // Test CspaceFreePolytope::FindRedundantInequalities.
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  Eigen::Matrix3d C;
  // clang-format off
  C << 1, 1, 0,
       0, 1, 1,
       1, 0, 1;
  // clang-format on
  Eigen::Vector3d d(2, 0.5, 0.1);
  Eigen::Vector3d s_lower(0, 0, 0);
  Eigen::Vector3d s_upper(0.5, 0.45, 1);
  std::unordered_set<int> C_redundant_indices;
  std::unordered_set<int> s_lower_redundant_indices;
  std::unordered_set<int> s_upper_redundant_indices;
  tester.FindRedundantInequalities(
      C, d, s_lower, s_upper, 0., &C_redundant_indices,
      &s_lower_redundant_indices, &s_upper_redundant_indices);
  EXPECT_EQ(C_redundant_indices, std::unordered_set<int>({0}));
  EXPECT_TRUE(s_lower_redundant_indices.empty());
  EXPECT_EQ(s_upper_redundant_indices, std::unordered_set<int>({0, 2}));
}

TEST_F(CIrisToyRobotTest, CalcDminusCs) {
  Eigen::Matrix<symbolic::Variable, 2, 3> C;
  Vector2<symbolic::Variable> d;
  for (int i = 0; i < 2; ++i) {
    d(i) = symbolic::Variable("d" + std::to_string(i));
    for (int j = 0; j < 3; ++j) {
      C(i, j) = symbolic::Variable(fmt::format("C{}{}", i, j));
    }
  }
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  const auto& s = tester.cspace_free_polytope().rational_forward_kin().s();

  const auto ret = tester.CalcDminusCs<symbolic::Variable>(C, d);
  EXPECT_EQ(ret.rows(), 2);
  for (int i = 0; i < 2; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, ret(i),
                 symbolic::Polynomial(
                     d(i) - C.row(i).dot(s.cast<symbolic::Expression>()),
                     symbolic::Variables(s)));
  }
}

TEST_F(CIrisToyRobotTest, CalcSBoundsPolynomial) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);
  VectorX<symbolic::Polynomial> s_minus_s_lower;
  VectorX<symbolic::Polynomial> s_upper_minus_s;
  EXPECT_EQ(tester.s_minus_s_lower().rows(), 3);
  EXPECT_EQ(tester.s_upper_minus_s().rows(), 3);
  const Eigen::Vector3d s_lower =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(

          plant_->GetPositionLowerLimits(), q_star);
  const Eigen::Vector3d s_upper =
      tester.cspace_free_polytope().rational_forward_kin().ComputeSValue(
          plant_->GetPositionUpperLimits(), q_star);
  const auto& s = tester.cspace_free_polytope().rational_forward_kin().s();
  for (int i = 0; i < 3; ++i) {
    EXPECT_PRED2(symbolic::test::PolyEqual, tester.s_minus_s_lower()(i),
                 symbolic::Polynomial((s(i) - s_lower(i))));
    EXPECT_PRED2(symbolic::test::PolyEqual, tester.s_upper_minus_s()(i),
                 symbolic::Polynomial((s_upper(i) - s(i))));
  }
}

TEST_F(CIrisToyRobotTest, CalcMonomialBasis) {
  // Test CalcMonomialBasis
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytope::Options options;
  for (bool with_cross_y : {false, true}) {
    options.with_cross_y = with_cross_y;
    CspaceFreePolytopeTester tester(
        plant_, scene_graph_, SeparatingPlaneOrder::kAffine, q_star, options);

    const auto& map_body_to_monomial_basis_array =
        tester.map_body_to_monomial_basis_array();
    // Make sure map_body_to_monomial_basis_array contains all pairs of bodies.
    for (const auto& plane :
         tester.cspace_free_polytope().separating_planes()) {
      for (const auto collision_geometry :
           {plane.positive_side_geometry, plane.negative_side_geometry}) {
        const SortedPair<multibody::BodyIndex> body_pair(
            plane.expressed_body, collision_geometry->body_index());
        auto it = map_body_to_monomial_basis_array.find(body_pair);
        EXPECT_NE(it, map_body_to_monomial_basis_array.end());
        const auto& monomial_basis_array = it->second;
        for (int i = 0; i < monomial_basis_array[0].rows(); ++i) {
          // Make sure the degree for each variable in the
          // monomial_basis_array[0] is at most 1.
          for (const auto& [var, degree] :
               monomial_basis_array[0](i).get_powers()) {
            EXPECT_LE(degree, 1);
          }
        }
        for (int i = 0; i < 3; ++i) {
          EXPECT_EQ(monomial_basis_array[i + 1].rows(),
                    monomial_basis_array[0].rows());
          for (int j = 0; j < monomial_basis_array[0].rows(); ++j) {
            EXPECT_EQ(
                monomial_basis_array[i + 1](j),
                symbolic::Monomial(tester.cspace_free_polytope().y_slack()(i)) *
                    monomial_basis_array[0](j));
          }
        }
      }
    }
  }
}

TEST_F(CIrisToyRobotTest, AddEllipsoidContainmentConstraint) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);

  solvers::MathematicalProgram prog;
  auto C = prog.NewContinuousVariables(8, 3);
  auto d = prog.NewContinuousVariables(8);
  auto ellipsoid_margins = prog.NewContinuousVariables(8);

  Eigen::Matrix3d Q;
  // Use arbitrary Q and s0
  // clang-format off
  Q << 1, 2, -1,
       0, 1, 2,
       2, -1, 3;
  // clang-format on
  const Eigen::Vector3d s0 = 0.4 * tester.s_lower() + tester.s_upper() * 0.6;
  tester.AddEllipsoidContainmentConstraint(&prog, Q, s0, C, d,
                                           ellipsoid_margins);
  prog.AddBoundingBoxConstraint(0, kInf, ellipsoid_margins);
  const auto result = solvers::Solve(prog);
  ASSERT_TRUE(result.is_success());
  const auto C_sol = result.GetSolution(C);
  const auto d_sol = result.GetSolution(d);
  const auto margin_sol = result.GetSolution(ellipsoid_margins);
  for (int i = 0; i < C_sol.rows(); ++i) {
    EXPECT_LE(C_sol.row(i).norm(), 1);
    EXPECT_LE((C_sol.row(i) * Q).norm(),
              d_sol(i) - C_sol.row(i).dot(s0) - margin_sol(i));
  }
}

TEST_F(CIrisToyRobotTest, AddCspacePolytopeContainment) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester tester(plant_, scene_graph_,
                                  SeparatingPlaneOrder::kAffine, q_star);

  Eigen::Matrix<double, 3, 4> s_inner_pts;
  // clang-format off
  s_inner_pts << 1, 2, 0, 3,
                 0, -1, 2, 3,
                 -1, 2, 1, 3;
  // clang-format on
  for (int i = 0; i < s_inner_pts.cols(); ++i) {
    s_inner_pts.col(i) = s_inner_pts.col(i)
                             .cwiseMin(tester.s_upper())
                             .cwiseMax(tester.s_lower());
  }
  solvers::MathematicalProgram prog;
  auto C = prog.NewContinuousVariables<5, 3>();
  auto d = prog.NewContinuousVariables<5>();
  tester.AddCspacePolytopeContainment(&prog, C, d, s_inner_pts);
  EXPECT_EQ(prog.linear_constraints().size(), 1);
  const VectorX<symbolic::Expression> constraint_val =
      prog.linear_constraints()[0].evaluator()->get_sparse_A() *
      prog.linear_constraints()[0].variables();
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->lower_bound(),
      Eigen::VectorXd::Constant(
          prog.linear_constraints()[0].evaluator()->num_constraints(), -kInf)));
  EXPECT_TRUE(CompareMatrices(
      prog.linear_constraints()[0].evaluator()->upper_bound(),
      Eigen::VectorXd::Constant(
          prog.linear_constraints()[0].evaluator()->num_constraints(), 0)));
  for (int i = 0; i < C.rows(); ++i) {
    for (int j = 0; j < s_inner_pts.cols(); ++j) {
      EXPECT_PRED2(symbolic::test::ExprEqual,
                   constraint_val(i * s_inner_pts.cols() + j),
                   C.row(i).dot(s_inner_pts.col(j)) - d(i));
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
