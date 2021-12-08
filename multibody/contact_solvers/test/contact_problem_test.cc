#include "drake/multibody/contact_solvers/contact_problem.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"

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
  std::vector<MatrixXd> A{S22, S33, S44, S22};
  VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  PRINT_VAR(A.size());
  PRINT_VAR(A[0].rows());
  PRINT_VAR(A[1].rows());
  PRINT_VAR(A[2].rows());
  PRINT_VAR(A[3].rows());
  SapContactProblem<double> problem(std::move(A), std::move(v_star));
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_velocities(), 11);

  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 1, J32, J33, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(1, 2, J33, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(2, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 2, J32, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 2, J32, J34, 1.0));
  EXPECT_EQ(problem.num_constraints(), 5);

  const ContactProblemGraph graph = problem.MakeGraph();
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_edges(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);

  PRINT_VAR(graph.num_cliques());
  PRINT_VAR(graph.num_edges());
  for (const auto& e : graph.edges()) {
    PRINT_VAR(e.cliques);
  }

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
  EXPECT_EQ(J.block_rows(), graph.num_edges());
  EXPECT_EQ(J.block_cols(), cliques_permutation.permuted_domain_size());

  PRINT_VAR(J.rows());
  PRINT_VAR(J.cols());
}

#if 0
GTEST_TEST(ContactProblem, Construction) {
  std::vector<MatrixXd> A{S22, S33, S44, S22};
  VectorXd v_star = VectorXd::LinSpaced(11, 1.0, 11.0);
  PRINT_VAR(A.size());
  PRINT_VAR(A[0].rows());
  PRINT_VAR(A[1].rows());
  PRINT_VAR(A[2].rows());
  PRINT_VAR(A[3].rows());
  SapContactProblem<double> problem(std::move(A), std::move(v_star));
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_velocities(), 11);

  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 1, J32, J33, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(1, 2, J33, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(2, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 2, J32, J34, 1.0));
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(0, 2, J32, J34, 1.0));
  EXPECT_EQ(problem.num_constraints(), 5);

  const ContactProblemGraph graph = problem.MakeGraph();
  EXPECT_EQ(graph.num_cliques(), 4);
  EXPECT_EQ(graph.num_edges(), 4);
  EXPECT_EQ(graph.num_constraints(), 5);

  PRINT_VAR(graph.num_cliques());
  PRINT_VAR(graph.num_edges());
  for (const auto& e : graph.edges()) {
    PRINT_VAR(e.cliques);
  }

  std::vector<int> participating_cliques;
  const ContactProblemGraph participating_cliques_graph =
      graph.MakeGraphOfParticipatingCliques(&participating_cliques);
  EXPECT_EQ(participating_cliques_graph.num_cliques(), 3);
  EXPECT_EQ(participating_cliques_graph.num_edges(), 4);
  EXPECT_EQ(participating_cliques_graph.num_constraints(), 5);
  EXPECT_EQ(participating_cliques.size(),
            participating_cliques_graph.num_cliques());

  PRINT_VAR(participating_cliques_graph.num_cliques());
  PRINT_VAR(participating_cliques_graph.num_edges());
  for (const auto& e : participating_cliques_graph.edges()) {
    PRINT_VAR(e.cliques);
  }
  PRINT_VAR(
      Eigen::Map<Eigen::Vector3i>(participating_cliques.data()).transpose());

  BlockSparseMatrix<double> J = MakeConstraintsBundleJacobian(
      problem, participating_cliques, participating_cliques_graph);
  EXPECT_EQ(J.rows(), 15);
  EXPECT_EQ(J.cols(), 9);
  EXPECT_EQ(J.block_rows(), participating_cliques_graph.num_edges());
  EXPECT_EQ(J.block_cols(), participating_cliques_graph.num_cliques());

  PRINT_VAR(J.rows());
  PRINT_VAR(J.cols());
}
#endif

}  // namespace
}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake