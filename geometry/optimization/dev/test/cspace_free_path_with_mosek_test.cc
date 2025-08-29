#include <memory>
#include <unordered_map>

#include <gtest/gtest.h>

#include "drake/common/polynomial.h"
#include "drake/common/trajectories/bezier_curve.h"
#include "drake/geometry/optimization/dev/cspace_free_path.h"
#include "drake/geometry/optimization/dev/test/c_iris_path_test_utilities.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"

namespace drake {
namespace geometry {
namespace optimization {

VectorX<Polynomiald> MakeBezierCurvePolynomialPath(
    const Eigen::Vector3d& s0, const Eigen::Vector3d& s_end, int curve_order,
    std::optional<HPolyhedron> poly_to_check_containment = std::nullopt) {
  if (poly_to_check_containment.has_value()) {
    EXPECT_TRUE(poly_to_check_containment.value().PointInSet(s0));
    EXPECT_TRUE(poly_to_check_containment.value().PointInSet(s_end));
  }
  VectorX<symbolic::Expression> s0_expr{3};
  s0_expr << s0(0), s0(1), s0(2);
  VectorX<symbolic::Expression> s_end_expr{3};
  s_end_expr << s_end(0), s_end(1), s_end(2);
  MatrixX<symbolic::Expression> control_points{3, curve_order + 1};

  control_points.col(0) = s0_expr;
  control_points.col(curve_order) = s_end_expr;
  const Eigen::Vector3d orth_offset{-0.01, 0.005, 0.005};
  for (int i = 1; i < curve_order - 1; ++i) {
    // another control point slightly off the straight line path between s0
    // and s_end.
    Eigen::Vector3d si = i * (s0 + s_end) / curve_order + orth_offset;
    if (poly_to_check_containment.has_value()) {
      EXPECT_TRUE(poly_to_check_containment.value().PointInSet(si));
    }
    VectorX<symbolic::Expression> si_expr{3};
    si_expr << si(0), si(1), si(2);
    control_points.col(i) = si_expr;
  }
  MatrixX<symbolic::Expression> control_points_expr{3, curve_order + 1};
  trajectories::BezierCurve<symbolic::Expression> path{0, 1, control_points};
  EXPECT_EQ(path.order(), curve_order);
  MatrixX<symbolic::Expression> bezier_path_expr =
      path.GetExpression(symbolic::Variable("t"));

  VectorX<Polynomiald> bezier_poly_path{bezier_path_expr.rows()};
  for (int r = 0; r < bezier_path_expr.rows(); ++r) {
    symbolic::Polynomial sym_poly{bezier_path_expr(r)};
    Eigen::VectorXd coefficients{sym_poly.TotalDegree() + 1};
    for (const auto& [monom, coeff] : sym_poly.monomial_to_coefficient_map()) {
      coefficients(monom.total_degree()) = coeff.Evaluate();
    }
    bezier_poly_path(r) = Polynomiald(coefficients);
  }
  return bezier_poly_path;
}

// @param a Maps the plane index to the separating plane parameter `a` in {x|
// aᵀx+b=0}
// @param b Maps the plane index to the separating plane parameter `b` in {x|
// aᵀx+b=0}
void CheckSeparationBySamples(
    const CspaceFreePathTester& tester, const systems::Diagram<double>& diagram,
    const Eigen::Ref<const Eigen::VectorXd>& mu_samples,
    const Eigen::Ref<const VectorX<Polynomiald>>& path,
    const std::unordered_map<int, Vector3<symbolic::Polynomial>>& a,
    const std::unordered_map<int, symbolic::Polynomial>& b,
    const Eigen::Ref<const Eigen::VectorXd>& q_star,
    const CspaceFreePolytope::IgnoredCollisionPairs& ignored_collision_pairs) {
  auto diagram_context = diagram.CreateDefaultContext();
  const auto& plant = tester.get_rational_forward_kin()->plant();
  const auto& scene_graph = tester.get_scene_graph();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());
  auto& scene_graph_context =
      scene_graph.GetMyMutableContextFromRoot(diagram_context.get());
  std::unique_ptr<AbstractValue> state_value =
      scene_graph.get_query_output_port().Allocate();

  for (int i = 0; i < mu_samples.rows(); ++i) {
    symbolic::Environment env;
    env.insert(tester.get_mu(), mu_samples(i));
    Eigen::VectorXd s{path.rows()};
    for (int r = 0; r < s.rows(); ++r) {
      s(r) = path(r).EvaluateUnivariate(mu_samples(i));
    }
    const Eigen::VectorXd q_val =
        tester.get_rational_forward_kin()->ComputeQValue(s, q_star);
    plant.SetPositions(&plant_context, q_val);
    const QueryObject<double>& query_object =
        state_value->get_value<QueryObject<double>>();
    scene_graph.get_query_output_port().Calc(scene_graph_context,
                                             state_value.get());
    EXPECT_FALSE(query_object.HasCollisions());
    for (int plane_index = 0;
         plane_index < ssize(tester.get_separating_planes()); ++plane_index) {
      const auto& plane = tester.get_separating_planes()[plane_index];
      if (!ignored_collision_pairs.contains(SortedPair<geometry::GeometryId>(
              plane.positive_side_geometry->id(),
              plane.negative_side_geometry->id())) &&
          a.contains(plane_index) && b.contains(plane_index)) {
        Eigen::Vector3d a_val;
        for (int j = 0; j < 3; ++j) {
          a_val(j) = a.at(plane_index)(j).Evaluate(env);
        }
        const double b_val = b.at(plane_index).Evaluate(env);
        EXPECT_GE(
            DistanceToHalfspace(*plane.positive_side_geometry, a_val, b_val,
                                plane.expressed_body, PlaneSide::kPositive,
                                plant, plant_context),
            0);
        EXPECT_GE(
            DistanceToHalfspace(*plane.negative_side_geometry, a_val, b_val,
                                plane.expressed_body, PlaneSide::kNegative,
                                plant, plant_context),
            0);
      }
    }
  }
}

void CheckForCollisionAlongPath(
    const CspaceFreePathTester& tester, const systems::Diagram<double>& diagram,
    const Eigen::Ref<const Eigen::VectorXd>& mu_samples,
    const Eigen::Ref<const VectorX<Polynomiald>>& path,
    const Eigen::Ref<const Eigen::VectorXd>& q_star) {
  auto diagram_context = diagram.CreateDefaultContext();
  const auto& plant = tester.get_rational_forward_kin()->plant();
  const auto& scene_graph = tester.get_scene_graph();
  auto& plant_context =
      plant.GetMyMutableContextFromRoot(diagram_context.get());
  auto& scene_graph_context =
      scene_graph.GetMyMutableContextFromRoot(diagram_context.get());
  std::unique_ptr<AbstractValue> state_value =
      scene_graph.get_query_output_port().Allocate();
  bool collision_found = false;
  for (int i = 0; i < mu_samples.rows(); ++i) {
    symbolic::Environment env;
    env.insert(tester.get_mu(), mu_samples(i));
    Eigen::VectorXd s{path.rows()};
    for (int r = 0; r < s.rows(); ++r) {
      s(r) = path(r).EvaluateUnivariate(mu_samples(i));
    }
    const Eigen::VectorXd q_val =
        tester.get_rational_forward_kin()->ComputeQValue(s, q_star);
    plant.SetPositions(&plant_context, q_val);
    const QueryObject<double>& query_object =
        state_value->get_value<QueryObject<double>>();
    scene_graph.get_query_output_port().Calc(scene_graph_context,
                                             state_value.get());
    collision_found = query_object.HasCollisions();
    if (collision_found) {
      break;
    }
  }
  EXPECT_TRUE(collision_found);
}

TEST_F(CIrisToyRobotTest, MakeAndSolveIsGeometrySeparableOnPathProgram) {
  const Eigen::Vector3d q_star(0, 0, 0);
  Eigen::Matrix<double, 9, 3> C_good;
  // clang-format off
  C_good << 1, 1, 0,
       -1, -1, 0,
       -1, 0, 1,
       1, 0, -1,
       0, 1, 1,
       0, -1, -1,
       1, 0, 1,
       1, 1, -1,
       1, -1, 1;
  // clang-format on
  Eigen::Matrix<double, 9, 1> d_good;
  d_good << 0.1, 0.1, 0.1, 0.02, 0.02, 0.2, 0.1, 0.1, 0.2;
  // This polyhedron is fully collision free and can be certified as such using
  // CspaceFreePolytope.
  const HPolyhedron c_free_polyhedron{C_good, d_good};
  const SortedPair<geometry::GeometryId> geometry_pair{body0_box_,
                                                       body2_sphere_};

  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions
      find_certificate_options{};
  find_certificate_options.verbose = false;
  solvers::MosekSolver solver;
  find_certificate_options.solver_id = solver.id();

  const int num_samples{100};
  Eigen::VectorXd mu_samples{100};
  mu_samples = Eigen::VectorXd::LinSpaced(num_samples, 0, 1);

  const Eigen::Vector3d s0_safe{0.06, -0.02, 0.04};
  const Eigen::Vector3d s_end_safe{-0.17, 0.08, -0.19};

  const Eigen::Vector3d s0_unsafe{-1.74, -0.22, 0.24};
  const Eigen::Vector3d s_end_unsafe{0.84, 2.31, 1.47};

  const int plane_order = 1;
  //  const Eigen::Vector3d s0_unsafe{0.65*M_PI, 1.63, 2.9};
  //  const Eigen::Vector3d s_end_unsafe{-1.63, 0.612, -0.65};

  for (const int maximum_path_degree : {1, 4}) {
    CspaceFreePathTester tester(plant_, scene_graph_, q_star,
                                maximum_path_degree, plane_order);

    // Check that we can certify paths up to the maximum degree.
    for (int bezier_curve_order = 1; bezier_curve_order <= maximum_path_degree;
         ++bezier_curve_order) {
      // Construct a polynonomial of degree bezier_curve_order <=
      // maximum_path_degree. By constructing this with the control points
      // inside c_free_polyhedron, we guarantee that the trajectory inside
      // will be colllision free.
      VectorX<Polynomiald> bezier_poly_path_safe =
          MakeBezierCurvePolynomialPath(s0_safe, s_end_safe, bezier_curve_order,
                                        c_free_polyhedron);
      auto separation_certificate_program =
          tester.cspace_free_path().MakeIsGeometrySeparableOnPathProgram(
              geometry_pair, bezier_poly_path_safe);
      auto separation_certificate_result =
          tester.cspace_free_path().SolveSeparationCertificateProgram(
              separation_certificate_program, find_certificate_options);
      EXPECT_TRUE(separation_certificate_result.result.is_success());
      if (separation_certificate_result.result.is_success()) {
        CheckSeparationBySamples(tester, *diagram_, mu_samples,
                                 bezier_poly_path_safe,
                                 {{separation_certificate_result.plane_index,
                                   separation_certificate_result.a}},
                                 {{separation_certificate_result.plane_index,
                                   separation_certificate_result.b}},
                                 q_star, {});
      }

      VectorX<Polynomiald> bezier_poly_path_unsafe =
          MakeBezierCurvePolynomialPath(s0_unsafe, s_end_unsafe,
                                        bezier_curve_order);
      auto separation_certificate_program2 =
          tester.cspace_free_path().MakeIsGeometrySeparableOnPathProgram(
              geometry_pair, bezier_poly_path_unsafe);

      auto separation_certificate_result2 =
          tester.cspace_free_path().SolveSeparationCertificateProgram(
              separation_certificate_program2, find_certificate_options);
      EXPECT_FALSE(separation_certificate_result2.result.is_success());
      if (!separation_certificate_result2.result.is_success()) {
        CheckForCollisionAlongPath(tester, *diagram_, mu_samples,
                                   bezier_poly_path_unsafe, q_star);
      }
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake
