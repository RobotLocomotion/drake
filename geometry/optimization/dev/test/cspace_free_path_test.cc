#include "drake/geometry/optimization/dev/cspace_free_path.h"

#include <optional>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/symbolic_test_util.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/optimization/dev/test/c_iris_test_utilities.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

TEST_F(CIrisToyRobotTest, CspaceFreePathConstructor) {
  // Test CspaceFreePolytope constructor.
  const Eigen::Vector3d q_star(0, 0, 0);
  for (unsigned int maximum_path_degree = 1; maximum_path_degree < 3;
       ++maximum_path_degree) {
    CspaceFreePathTester tester(plant_, scene_graph_,
                                SeparatingPlaneOrder::kAffine, q_star,
                                maximum_path_degree);
    // check that the path map is properly instantiated
    for (const auto& s_set_itr : tester.get_s_set()) {
      EXPECT_GT(tester.get_path().count(s_set_itr), 0);
      const symbolic::Polynomial& poly{tester.get_path().at(s_set_itr)};
      EXPECT_EQ(poly.indeterminates().size(), 1);
      EXPECT_EQ((*poly.indeterminates().cbegin()), tester.get_mu());
      EXPECT_EQ(static_cast<unsigned int>(poly.TotalDegree()),
                maximum_path_degree);
      EXPECT_EQ(poly.monomial_to_coefficient_map().size(),
                maximum_path_degree + 1);
    }
  }
}

TEST_F(CIrisToyRobotTest, CspaceFreePathGeneratePathRationalsTest) {
  const Eigen::Vector3d q_star(0, 0, 0);
  CspaceFreePolytopeTester polytope_tester(
      plant_, scene_graph_, SeparatingPlaneOrder::kAffine, q_star);
  auto find_free_polytope_plane_geometry_with_plane_index =
      [&polytope_tester](
          int plane_index) -> std::optional<PlaneSeparatesGeometries> {
    for (const auto& plane_geometry :
         polytope_tester.get_mutable_plane_geometries()) {
      if (plane_geometry.plane_index == plane_index) {
        return std::optional{plane_geometry};
      }
    }
    return std::nullopt;
  };

  for (unsigned int maximum_path_degree = 1; maximum_path_degree < 3;
       ++maximum_path_degree) {
    CspaceFreePathTester tester(plant_, scene_graph_,
                                SeparatingPlaneOrder::kAffine, q_star,
                                maximum_path_degree);
    EXPECT_EQ(tester.get_mutable_plane_geometries().size(),
              tester.cspace_free_path().separating_planes().size());
    const symbolic::Variables mu_indets{tester.get_mu()};
    const symbolic::Variables y_slack{tester.cspace_free_path().y_slack()};

    for (const auto& free_path_plane_geometry :
         tester.get_mutable_plane_geometries()) {
      const auto& plane =
          tester.cspace_free_path()
              .separating_planes()[free_path_plane_geometry.plane_index];
      const auto& free_polytope_plane_geometry =
          find_free_polytope_plane_geometry_with_plane_index(
              free_path_plane_geometry.plane_index);
      EXPECT_TRUE(free_polytope_plane_geometry.has_value());

      const auto& positive_side =
          std::tie(free_path_plane_geometry.positive_side_rationals,
                   free_polytope_plane_geometry.value().positive_side_rationals,
                   plane.positive_side_geometry);
      const auto& negative_side =
          std::tie(free_path_plane_geometry.negative_side_rationals,
                   free_polytope_plane_geometry.value().negative_side_rationals,
                   plane.negative_side_geometry);
      for (const auto& [path_rationals, polytope_rationals, geometry] :
           {positive_side, negative_side}) {
        EXPECT_EQ(path_rationals.size(), polytope_rationals.size());
        for (int i = 0; i < static_cast<int>(path_rationals.size()); ++i) {
          const symbolic::RationalFunction& path_rational{path_rationals.at(i)};
          const symbolic::RationalFunction& polytope_rational{
              polytope_rationals.at(i)};

          // The denominator is only a function of the new path variable or is
          // constant.
          EXPECT_LE(path_rational.denominator().indeterminates().size(), 1);
          if (path_rational.denominator().indeterminates().size() == 1) {
            EXPECT_EQ(path_rational.denominator().indeterminates(), mu_indets);
          }

          const symbolic::Polynomial& path_numerator{path_rational.numerator()};

          // Gets the y_slacks needed to implement this matrix SOS condition.
          int num_y_slack =
              intersect(
                  symbolic::Variables{
                      polytope_tester.cspace_free_polytope().y_slack()},
                  polytope_rational.numerator().indeterminates())
                  .size();
          const symbolic::Variables cur_y_slacks{
              tester.cspace_free_path().y_slack().topRows(num_y_slack)};
          symbolic::Variables mu_and_y_indets{tester.get_mu()};
          mu_and_y_indets.insert(cur_y_slacks.begin(), cur_y_slacks.end());

          // The numerator is a function of the new path variable and of the
          // matrix SOS variables.
          EXPECT_EQ(path_numerator.indeterminates().size(),
                    cur_y_slacks.size() + 1);
          EXPECT_EQ(path_numerator.indeterminates(), mu_and_y_indets);

          for (const auto& y : cur_y_slacks) {
            EXPECT_EQ(path_numerator.Degree(y), 2);
          }
          auto compute_total_s_degree =
              [&polytope_tester](const symbolic::Polynomial& poly) {
                int degree{0};
                for (const auto& [m, c] : poly.monomial_to_coefficient_map()) {
                  int cur_s_degree{0};
                  for (const auto& s : polytope_tester.get_s_set()) {
                    cur_s_degree += m.degree(s);
                  }
                  degree = std::max(degree, cur_s_degree);
                }
                return degree;
              };
          // The degree in terms of the path variable should be the total degree
          // of the original rational with respect to the variables s_set_ times
          // the path degree.
          EXPECT_EQ(
              static_cast<unsigned int>(path_numerator.Degree(tester.get_mu())),
              maximum_path_degree *
                  compute_total_s_degree(polytope_rational.numerator()));
        }
      }
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
