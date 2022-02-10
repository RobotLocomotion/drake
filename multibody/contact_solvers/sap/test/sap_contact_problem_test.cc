#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <gtest/gtest.h>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

// Useful SPD matrices for testing. The numbers specify their size.
// clang-format off
const Eigen::Matrix2d S22 =
    (Eigen::Matrix2d() << 2, 1,
                          1, 2).finished();
const Eigen::Matrix3d S33 =
    (Eigen::Matrix3d() << 4, 1, 2,
                          1, 5, 3,
                          2, 3, 6).finished();
const Eigen::Matrix4d S44 =
    (Eigen::Matrix4d() << 7, 1, 2, 3,
                          1, 8, 4, 5,
                          2, 4, 9, 6,
                          3, 5, 6, 10).finished();
// clang-format on

// Unit test the construction of a SapContactProblem.
GTEST_TEST(ContactProblem, Construction) {
  const double time_step = 0.01;
  const std::vector<MatrixXd> A{S22, S33, S44, S22};
  const VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  SapContactProblem<double> problem(time_step, A, v_star);
  EXPECT_EQ(problem.time_step(), time_step);
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_velocities(), 11);
  EXPECT_EQ(problem.num_velocities(0), 2);
  EXPECT_EQ(problem.num_velocities(1), 3);
  EXPECT_EQ(problem.num_velocities(2), 4);
  EXPECT_EQ(problem.num_velocities(3), 2);
  EXPECT_EQ(problem.dynamics_matrix(), A);
  EXPECT_EQ(problem.v_star(), v_star);
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
