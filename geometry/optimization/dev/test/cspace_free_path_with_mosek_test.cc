#include <iostream>

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
    const Eigen::Vector<double, 3>& s0, const Eigen::Vector<double, 3>& s_end,
    int curve_order,
    std::optional<HPolyhedron> poly_to_check_containment = std::nullopt) {
  if (poly_to_check_containment.has_value()) {
    EXPECT_TRUE(poly_to_check_containment.value().PointInSet(s0));
    EXPECT_TRUE(poly_to_check_containment.value().PointInSet(s_end));
  }
  VectorX<symbolic::Expression> s0_expr{3};
  s0_expr << s0(0), s0(1), s0(2);
  VectorX<symbolic::Expression> s_end_expr{3};
  s_end_expr << s_end(0), s_end(1), s_end(2);
  Eigen::MatrixX<symbolic::Expression> control_points{3, curve_order + 1};

  control_points.col(0) = s0_expr;
  control_points.col(curve_order) = s_end_expr;
  const Eigen::Vector<double, 3> orth_offset{-0.01, 0.005, 0.005};
  for (int i = 1; i < curve_order - 1; ++i) {
    // another control point slightly off the straight line path between s0
    // and s_end.
    Eigen::Vector<double, 3> si = i * (s0 + s_end) / curve_order + orth_offset;
    if (poly_to_check_containment.has_value()) {
      EXPECT_TRUE(poly_to_check_containment.value().PointInSet(si));
    }
    VectorX<symbolic::Expression> si_expr{3};
    si_expr << si(0), si(1), si(2);
    control_points.col(i) = si_expr;
  }
  Eigen::MatrixX<symbolic::Expression> control_points_expr{3, curve_order + 1};
  trajectories::BezierCurve<symbolic::Expression> path{0, 1, control_points};
  EXPECT_EQ(path.order(), curve_order);
  Eigen::MatrixX<symbolic::Expression> bezier_path_expr =
      path.value(symbolic::Variable("t"));

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
  d_good << 0.1, 0.2, 0.3, 0.2, 0.2, 0.2, 0.1, 0.1, 0.2;
  // This polyhedron is fully collision free and can be certified as such using
  // CspaceFreePolytope.
  const HPolyhedron c_free_polyhedron{C_good, d_good};
  const SortedPair<geometry::GeometryId> geometry_pair{body0_box_,
                                                       body2_sphere_};
  const Eigen::Vector<double, 3> s0_safe{0.05, 0.05, 0.05};
  const Eigen::Vector<double, 3> s_end_safe{-0.05, -0.05, -0.05};

  const Eigen::Vector<double, 3> s0_unsafe{5, 5, 5};
  const Eigen::Vector<double, 3> s_end_unsafe{-0.5, -0.5, -0.5};

  CspaceFreePolytope::FindSeparationCertificateGivenPolytopeOptions
      find_certificate_options;
  find_certificate_options.verbose = false;
  solvers::MosekSolver solver;
  find_certificate_options.solver_id = solver.id();

  for (const int maximum_path_degree : {1, 3}) {
    CspaceFreePathTester tester(plant_, scene_graph_,
                                SeparatingPlaneOrder::kAffine, q_star,
                                maximum_path_degree);
    // Check that we can certify paths up to the maximum degree.
    for (int bezier_curve_order = 1; bezier_curve_order <= maximum_path_degree;
         ++bezier_curve_order) {
      // Construct a polynonomial of degree bezier_curve_order <=
      // maximum_path_degree. By constructing this with the control points
      // inside c_free_polyhedron, we guarantee that the trajectory inside will
      // be colllision free.
      VectorX<Polynomiald> bezier_poly_path_safe =
          MakeBezierCurvePolynomialPath(s0_safe, s_end_safe, bezier_curve_order,
                                        c_free_polyhedron);
      auto separation_certificate_program =
          tester.cspace_free_path().MakeIsGeometrySeparableOnPathProgram(
              geometry_pair, bezier_poly_path_safe);
      auto separation_certificate_result =
          tester.cspace_free_path().SolveSeparationCertificateProgram(
              separation_certificate_program, find_certificate_options);
      EXPECT_TRUE(separation_certificate_result.has_value());

      VectorX<Polynomiald> bezier_poly_path_unsafe =
          MakeBezierCurvePolynomialPath(s0_unsafe, s_end_unsafe,
                                        bezier_curve_order);
      separation_certificate_program =
          tester.cspace_free_path().MakeIsGeometrySeparableOnPathProgram(
              geometry_pair, bezier_poly_path_safe);
      separation_certificate_result =
          tester.cspace_free_path().SolveSeparationCertificateProgram(
              separation_certificate_program, find_certificate_options);
      // TODO: For now this is always feasible which is not what we want. It might be
      // that these bodies are always collision free? Need to find a collision!
      EXPECT_FALSE(separation_certificate_result.has_value());
    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake

// int main(int argc, char** argv) {
//  // Ensure that we have the MOSEK license for the entire duration of this
//  test,
//  // so that we do not have to release and re-acquire the license for every
//  // test.
//  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
//  ::testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
//}