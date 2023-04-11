#include <gtest/gtest.h>

#include <iostream>

#include "drake/geometry/optimization/dev/cspace_free_path.h"
#include "drake/geometry/optimization/dev/test/c_iris_path_test_utilities.h"
#include "drake/geometry/optimization/test/c_iris_test_utilities.h"
#include "drake/solvers/common_solver_option.h"
#include "drake/solvers/mosek_solver.h"
#include "drake/solvers/solve.h"
#include "drake/common/trajectories/bezier_curve.h"



namespace drake {
namespace geometry {
namespace optimization {

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
  const Eigen::Vector<double, 3> s0{0.05, 0.05, 0.05};
  const Eigen::Vector<double, 3> s_end{-0.05, -0.05, -0.05};
  ASSERT_TRUE(c_free_polyhedron.PointInSet(s0));
  ASSERT_TRUE(c_free_polyhedron.PointInSet(s_end));

  for (const int maximum_path_degree : {1, 3}) {
    CspaceFreePathTester tester(plant_, scene_graph_,
                                SeparatingPlaneOrder::kAffine, q_star,
                                maximum_path_degree);
    for (int bezier_curve_order = 1; bezier_curve_order <= maximum_path_degree; ++bezier_curve_order) {
        // Construct a polynonomial of degree bezier_curve_order <= maximum_path_degree.
        Eigen::MatrixXd control_points{3, bezier_curve_order + 1};
        control_points.col(0) = s0;
        control_points.col(bezier_curve_order) = s_end;
        const Eigen::Vector<double, 3> orth_offset{-0.01, 0.005, 0.005};
        for(int i = 1; i < bezier_curve_order - 1; ++i) {
          // another control point slightly off the straight line path between s0 and s_end.
          Eigen::Vector<double, 3> si = i*(s0 + s_end)/bezier_curve_order + orth_offset;
          ASSERT_TRUE(c_free_polyhedron.PointInSet(si));
        }
        trajectories::BezierCurve<double> path{0,1,control_points};
        EXPECT_EQ(path.order(), bezier_curve_order);

    }
  }
}

}  // namespace optimization
}  // namespace geometry
}  // namespace drake

//int main(int argc, char** argv) {
//  // Ensure that we have the MOSEK license for the entire duration of this test,
//  // so that we do not have to release and re-acquire the license for every
//  // test.
//  auto mosek_license = drake::solvers::MosekSolver::AcquireLicense();
//  ::testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();
//}