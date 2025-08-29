#include "drake/geometry/optimization/dev/cspace_free_path.h"

#include <algorithm>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/optimization/cspace_free_internal.h"
#include "drake/geometry/optimization/dev/test/c_iris_path_test_utilities.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/multibody/rational/rational_forward_kinematics_internal.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

TEST_F(CIrisToyRobotTest, CspaceFreePathConstructor) {
  // Test CspaceFreePolytope constructor.
  const Eigen::Vector3d q_star(0, 0, 0);
  for (int maximum_path_degree = 1; maximum_path_degree < 3;
       ++maximum_path_degree) {
    for (int plane_degree = 1; plane_degree < 3; ++plane_degree) {
      CspaceFreePathTester tester(plant_, scene_graph_, q_star,
                                  maximum_path_degree, plane_degree);
      const CspaceFreePath& dut = tester.cspace_free_path();
      EXPECT_EQ(dut.max_degree(), maximum_path_degree);
      EXPECT_EQ(dut.plane_degree(), plane_degree);
      // check that the path map is properly instantiated
      VectorX<symbolic::Variable> s_vars{
          tester.get_rational_forward_kin()->s()};
      for (int i = 0; i < s_vars.size(); ++i) {
        EXPECT_TRUE(tester.get_path().contains(s_vars(i)));

        const symbolic::Polynomial& path_component{
            tester.get_path().at(s_vars(i))};
        EXPECT_EQ(path_component.indeterminates().size(), 1);
        EXPECT_EQ((*path_component.indeterminates().cbegin()), tester.get_mu());
        EXPECT_EQ(static_cast<unsigned int>(path_component.TotalDegree()),
                  maximum_path_degree);
        EXPECT_EQ(path_component.monomial_to_coefficient_map().size(),
                  maximum_path_degree + 1);
      }

      int num_planes_expected = 0;

      const auto link_geometries =
          internal::GetCollisionGeometries(*plant_, *scene_graph_);
      // Count the expected number of planes by hand.
      num_planes_expected +=
          link_geometries.at(plant_->world_body().index()).size() *
          // Don't include world_body to body0 as there is only a weld joint
          // between them.
          (link_geometries.at(body_indices_[1]).size() +
           link_geometries.at(body_indices_[2]).size() +
           link_geometries.at(body_indices_[3]).size());
      num_planes_expected += link_geometries.at(body_indices_[0]).size() *
                             link_geometries.at(body_indices_[2]).size();
      num_planes_expected += link_geometries.at(body_indices_[1]).size() *
                             link_geometries.at(body_indices_[3]).size();
      num_planes_expected += link_geometries.at(body_indices_[2]).size() *
                             link_geometries.at(body_indices_[3]).size();
      EXPECT_EQ(tester.get_separating_planes().size(), num_planes_expected);
      EXPECT_EQ(dut.plane_degree(), plane_degree);
      for (const auto& [geometry_pair, plane_index] :
           dut.map_geometries_to_separating_planes()) {
        // check plane
        const auto& plane = dut.separating_planes()[plane_index];
        EXPECT_EQ(plane.plane_degree, plane_degree);
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
          EXPECT_EQ(plane.a(i).TotalDegree(), plane_degree);
          EXPECT_EQ(plane.a(i).indeterminates(),
                    symbolic::Variables({dut.mu()}));
        }
        EXPECT_EQ(plane.b.TotalDegree(), plane_degree);
        EXPECT_EQ(plane.b.indeterminates(), symbolic::Variables({dut.mu()}));
      }
      EXPECT_EQ(dut.y_slack().size(), 3);
      // We will check the path rationals in a later test.
    }
  }
}

namespace {
// Checks whether two polynomials with potentially different variable ids are
// the same by checking whether setting all their variables to the same value
// makes the two polynomials evaluate to the same value.
bool PolyEqualBySameRandomEval(symbolic::Polynomial p1, symbolic::Polynomial p2,
                               double value, double tol) {
  auto evaluate_at_value = [&value](const symbolic::Polynomial& p) {
    symbolic::Environment env;
    for (const auto& var : p.indeterminates()) {
      env.insert(var, value);
    }
    for (const auto& var : p.decision_variables()) {
      env.insert(var, value);
    }
    return p.Evaluate(env);
  };
  const double eval1 = evaluate_at_value(p1);
  const double eval2 = evaluate_at_value(p2);
  return std::abs(eval1 - eval2) / std::abs(eval1) < tol;
}
}  // namespace

TEST_F(CIrisToyRobotTest, CspaceFreePathGeneratePathRationalsTest) {
  const Eigen::Vector3d q_star(0, 0, 0);
  // A random generator with fixed seed that will be used to check whether two
  // polynomials are equal.
  std::default_random_engine generator;
  generator.seed(0);
  std::uniform_real_distribution<> val_generator(-1.5, 1.5);
  const Eigen::Vector4d mu_test_values{0, 0.25, 0.77, 1};
  const int maximum_path_degree = 3;
  const int plane_degree = 2;

  CspaceFreePathTester tester(plant_, scene_graph_, q_star, maximum_path_degree,
                              plane_degree);

  const CspaceFreePath& dut = tester.cspace_free_path();
  EXPECT_EQ(tester.get_path_plane_geometries().size(),
            dut.separating_planes().size());

  const symbolic::Variables mu_indets{tester.get_mu()};
  const symbolic::Variables y_slack{tester.cspace_free_path().y_slack()};

  std::vector<std::unique_ptr<CSpaceSeparatingPlane<symbolic::Variable>>>
      separating_planes_ptrs;
  separating_planes_ptrs.reserve(dut.separating_planes().size());
  for (const auto& plane : dut.separating_planes()) {
    separating_planes_ptrs.push_back(
        std::make_unique<CSpaceSeparatingPlane<symbolic::Variable>>(plane));
  }

  // We should have the same number of path_plane_geometries as
  // internal::GenerateRationals generates.
  std::vector<PlaneSeparatesGeometries> plane_geometries_in_s;
  internal::GenerateRationals(separating_planes_ptrs, dut.y_slack(), q_star,
                              *(tester.get_rational_forward_kin()),
                              &plane_geometries_in_s);

  EXPECT_EQ(plane_geometries_in_s.size(),
            tester.get_path_plane_geometries().size());

  for (const auto& plane_geometry : tester.get_path_plane_geometries()) {
    const auto& plane = dut.separating_planes()[plane_geometry.plane_index];

    if (plane.positive_side_geometry->type() == CIrisGeometryType::kPolytope &&
        plane.negative_side_geometry->type() == CIrisGeometryType::kPolytope) {
      EXPECT_EQ(plane_geometry.positive_side_conditions.size(),
                plane.positive_side_geometry->num_rationals());
      EXPECT_EQ(plane_geometry.negative_side_conditions.size(),
                plane.negative_side_geometry->num_rationals());
    } else if (plane.positive_side_geometry->type() ==
                   CIrisGeometryType::kPolytope &&
               plane.negative_side_geometry->type() !=
                   CIrisGeometryType::kPolytope) {
      EXPECT_EQ(plane_geometry.positive_side_conditions.size(),
                plane.positive_side_geometry->num_rationals());
      EXPECT_EQ(plane_geometry.negative_side_conditions.size(),
                plane.negative_side_geometry->num_rationals() - 1);
    } else if (plane.positive_side_geometry->type() !=
                   CIrisGeometryType::kPolytope &&
               plane.negative_side_geometry->type() ==
                   CIrisGeometryType::kPolytope) {
      EXPECT_EQ(plane_geometry.positive_side_conditions.size(),
                plane.positive_side_geometry->num_rationals() - 1);
      EXPECT_EQ(plane_geometry.negative_side_conditions.size(),
                plane.negative_side_geometry->num_rationals());
    } else {
      EXPECT_EQ(plane_geometry.positive_side_conditions.size(),
                plane.positive_side_geometry->num_rationals());
      EXPECT_EQ(plane_geometry.negative_side_conditions.size(),
                plane.negative_side_geometry->num_rationals() - 1);
    }

    const auto& plane_geometry_s =
        plane_geometries_in_s.at(plane_geometry.plane_index);

    const auto& positive_side =
        std::tie(plane_geometry.positive_side_conditions,
                 plane_geometry_s.positive_side_rationals);
    const auto& negative_side =
        std::tie(plane_geometry.negative_side_conditions,
                 plane_geometry_s.negative_side_rationals);

    for (const auto& [path_conditions, polytope_rationals] :
         {positive_side, negative_side}) {
      EXPECT_EQ(path_conditions.size(), polytope_rationals.size());
      int i = 0;
      // Path rationals is a list while polytope_rationals is a vector so we
      // iterate through the list and increment a counter for the random
      // access to the vector.
      for (const auto& path_rational : path_conditions) {
        const symbolic::Polynomial& path_condition{path_rational.get_poly()};
        const symbolic::Polynomial& polytope_condition{
            polytope_rationals.at(i).numerator()};
        ++i;

        // Gets the y_slacks needed to implement this matrix SOS condition.
        const symbolic::Variables path_y_slack =
            intersect(y_slack, path_condition.indeterminates());
        const symbolic::Variables polytope_y_slack =
            intersect(y_slack, polytope_condition.indeterminates());
        // The same y_slacks should be used to implement both the polytope
        // and path condition.
        EXPECT_EQ(path_y_slack, polytope_y_slack);

        symbolic::Variables mu_and_y_indets{tester.get_mu()};
        mu_and_y_indets.insert(path_y_slack.begin(), path_y_slack.end());

        // The numerator is a function of the new path variable and of the
        // matrix SOS variables.
        EXPECT_EQ(path_condition.indeterminates(), mu_and_y_indets);

        // The condition should be a quadratic in the y_slack.
        for (const auto& y : path_y_slack) {
          EXPECT_EQ(path_condition.Degree(y), 2);
        }

        auto compute_total_s_degree =
            [&tester](const symbolic::Polynomial& poly) {
              int degree{0};
              for (const auto& [m, c] : poly.monomial_to_coefficient_map()) {
                int cur_s_degree{0};
                for (int k = 0;
                     k < tester.get_rational_forward_kin()->s().size(); ++k) {
                  cur_s_degree +=
                      m.degree(tester.get_rational_forward_kin()->s()(k));
                }
                degree = std::max(degree, cur_s_degree);
              }
              return degree;
            };
        // The degree in terms of the path variable should be the total
        // degree of the original rational with respect to the variables
        // s_set_ plus the path degree times the plane_degree
        EXPECT_EQ(
            static_cast<unsigned int>(path_condition.Degree(tester.get_mu())),
            maximum_path_degree * compute_total_s_degree(polytope_condition) +
                plane_degree);

        // Now check that the path_conditions are actually properly
        // substituted.
        symbolic::Substitution path_sub;
        for (const auto& [var, poly] : tester.get_path()) {
          path_sub.insert({var, poly.ToExpression()});
        }
        for (int j = 0; j < mu_test_values.rows(); ++j) {
          symbolic::Environment mu_env;
          mu_env.insert(tester.get_mu(), mu_test_values(j));
          const symbolic::Polynomial polytope_condition_sub_path{
              polytope_condition.ToExpression().Substitute(path_sub)};
          for (int k = 0; k < 10; ++k) {
            EXPECT_TRUE(PolyEqualBySameRandomEval(
                path_condition.EvaluatePartial(mu_env),
                polytope_condition_sub_path.EvaluatePartial(mu_env),
                val_generator(generator), 1e-10));
          }
        }
      }
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
