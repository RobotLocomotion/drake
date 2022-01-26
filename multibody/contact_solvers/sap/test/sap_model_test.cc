#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"

#define PRINT_VAR(a) std::cout << #a ": " << a << std::endl;
#define PRINT_VARn(a) std::cout << #a ":\n" << a << std::endl;

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
const MatrixXd J31 =
    (MatrixXd(3, 1) << 1,
                       2,
                       3).finished();

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

GTEST_TEST(SapModel, MakeConstraintsBundleJacobian) {
  const double time_step = 0.01;
  const int num_cliques = 4;
  // Initialize the dynamics matrix to A = {I₁, 2I₂, 3I₃, 4I₄}.
  // Therefore the number of generalized velocities is nv = 10.
  std::vector<MatrixXd> A;
  A.resize(num_cliques);
  for (int c = 1; c <= num_cliques; ++c) {
    A[c - 1].resize(c, c);
    A[c - 1] = MatrixXd::Identity(c, c) * c;
  }
  const int num_velocities = 10;
  VectorXd v_star = VectorXd::LinSpaced(num_velocities, 1.0, 10.0);

  // N.B. We create copies of A and v_star so that when moved in the constructor
  // arguments the original objects remain valid in this scope.
  SapContactProblem<double> problem(time_step, std::vector<MatrixXd>(A),
                                    VectorXd(v_star));
  EXPECT_EQ(problem.num_cliques(), 4);
  EXPECT_EQ(problem.num_constraints(), 0);
  EXPECT_EQ(problem.num_velocities(), num_velocities);
  EXPECT_EQ(problem.num_velocities(0), 1);
  EXPECT_EQ(problem.num_velocities(1), 2);
  EXPECT_EQ(problem.num_velocities(2), 3);
  EXPECT_EQ(problem.num_velocities(3), 4);

  PRINT_VAR(problem.num_cliques());
  PRINT_VAR(problem.num_velocities());

  // Make a problem using contact constraints with the following graph:
  //
  //                       2
  //                      ┌─┐
  //                      │ │
  // ┌───┐  0  ┌───┐  1  ┌┴─┴┐     ┌───┐
  // │ 0 ├─────┤ 1 ├─────┤ 2 │     │ 3 │
  // └─┬─┘     └───┘     └─┬─┘     └───┘
  //   │                   │
  //   └───────────────────┘
  //           3,4
  //

  // Contact constraint properties:
  const double mu = 1.0;        // Friction coefficient.
  const double k = 1.0e10;      // Stiffness.
  const double taud = 0.0;      // Dissipation time scale.
  const double phi0 = -1.0e-3;  // Signed distance.
  const SapFrictionConeConstraint<double>::Parameters parameters{mu, k, taud,
                                                                 phi0};
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 1, J31, J32, parameters));
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      1, 2, J32, J33, parameters));  
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(2, J33, parameters));        
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 2, J31, J33, parameters));
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 2, J31, J33, parameters));        

  PRINT_VAR(problem.num_constraints());
  PRINT_VAR(problem.num_constrained_dofs());            
  EXPECT_EQ(problem.num_constraints(), 5);
  EXPECT_EQ(problem.num_constrained_dofs(), 15);

  const SapModel<double> model(&problem);
  EXPECT_EQ(model.num_cliques(), 4);
  EXPECT_EQ(model.num_participating_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), num_velocities);
  EXPECT_EQ(model.num_constraints(), 5);
  EXPECT_EQ(model.num_participating_velocities(), 6);
  EXPECT_EQ(model.num_impulses(), 15);

  PRINT_VAR(model.num_cliques());
  PRINT_VAR(model.num_participating_cliques());

  for (const auto& Ac : model.dynamics_matrix()) {
    PRINT_VARn(Ac);
  }

  PRINT_VAR(model.num_participating_cliques());

  std::vector<MatrixXd> A_permuted_expected;
  A_permuted_expected.push_back(A[0]);
  A_permuted_expected.push_back(A[1]);
  A_permuted_expected.push_back(A[2]);

  for (const auto& Ac : A_permuted_expected) {
    PRINT_VARn(Ac);
  }

  EXPECT_EQ(model.dynamics_matrix(), A_permuted_expected);

  PRINT_VAR(model.num_cliques());
  PRINT_VAR(model.num_participating_cliques());
  PRINT_VAR(model.num_velocities());
  PRINT_VAR(model.num_constraints());
  PRINT_VAR(model.num_participating_velocities());
  PRINT_VAR(model.num_impulses());

#if 0
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
#endif  

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