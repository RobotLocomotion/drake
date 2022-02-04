#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {
namespace {

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

// clang-format off
const MatrixXd J32 =
    (MatrixXd(3, 2) << 2, 1,
                       1, 2,
                       1, 2).finished();
const MatrixXd J33 =
    (MatrixXd(3, 3) << 4, 1, 2,
                       1, 5, 3,
                       2, 3, 6).finished();
const MatrixXd J34 =
    (MatrixXd(3, 4) << 7, 1, 2, 3,
                       1, 8, 4, 5,
                       2, 4, 9, 6).finished();
// clang-format on

GTEST_TEST(ContactProblem, MakeConstraintsBundleJacobian) {
  const double time_step = 0.01;
  std::vector<MatrixXd> A{S22, S33, S44, S22};
  VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  PRINT_VAR(A.size());
  PRINT_VAR(A[0].rows());
  PRINT_VAR(A[1].rows());
  PRINT_VAR(A[2].rows());
  PRINT_VAR(A[3].rows());
  SapContactProblem<double> problem(time_step, std::move(A), std::move(v_star));
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_velocities(), 11);

  const double mu = 1.0;
  const double k = 1.0e10;
  const double taud = 0.0;
  const double phi0 = -1.0e-3;
  const SapFrictionConeConstraint<double>::Parameters parameters{mu, k, taud,
                                                                 phi0};

  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 1, J32, J33, parameters));
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      1, 2, J33, J34, parameters));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(2, J34, parameters));
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 2, J32, J34, parameters));
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 2, J32, J34, parameters));
  EXPECT_EQ(problem.num_constraints(), 5);
  EXPECT_EQ(problem.num_constrained_dofs(), 15);
  EXPECT_EQ(problem.num_velocities(), 11);
  EXPECT_EQ(problem.num_velocities(0), 2);
  EXPECT_EQ(problem.num_velocities(1), 3);
  EXPECT_EQ(problem.num_velocities(2), 4);
  EXPECT_EQ(problem.num_velocities(3), 2);

  // Unit test MakeGraph().
  const ContactProblemGraph graph = problem.MakeGraph();
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_constraint_groups(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);

  PRINT_VAR(graph.num_cliques());
  PRINT_VAR(graph.num_constraint_groups());
  for (const auto& e : graph.constraint_groups()) {
    PRINT_VAR(e.cliques);
  }

  // TODO: This below should be part of the SapModel unit tests.
#if 0
  const PartialPermutation cliques_permutation =
      MakeParticipatingCliquesPermutation(graph);
  EXPECT_EQ(cliques_permutation.domain_size(), graph.num_cliques());
  EXPECT_EQ(cliques_permutation.permuted_domain_size(), 3);

  PRINT_VAR(cliques_permutation.domain_size());
  PRINT_VAR(cliques_permutation.permuted_domain_size());
  const BlockSparseMatrix<double> J =
      MakeConstraintsBundleJacobian(problem, graph, cliques_permutation);

  EXPECT_EQ(J.rows(), 15);
  EXPECT_EQ(J.cols(), 9);
  EXPECT_EQ(J.block_rows(), graph.num_constraint_groups());
  EXPECT_EQ(J.block_cols(), cliques_permutation.permuted_domain_size());

  PRINT_VAR(J.rows());
  PRINT_VAR(J.cols());
#endif
}

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake