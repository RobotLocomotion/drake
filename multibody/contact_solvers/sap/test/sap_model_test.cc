#include "drake/multibody/contact_solvers/sap/sap_model.h"

#include <iostream>

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/common/test_utilities/expect_throws_message.h"
#include "drake/multibody/contact_solvers/sap/sap_contact_problem.h"
#include "drake/multibody/contact_solvers/sap/contact_problem_graph.h"

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

const MatrixXd Z31 = MatrixXd::Zero(3, 1);
const MatrixXd Z32 = MatrixXd::Zero(3, 2);
const MatrixXd Z34 = MatrixXd::Zero(3, 4);

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
  // │ 0 ├─────┤ 1 ├─────┤ 3 │     │ 2 │
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
      1, 3, J32, J34, parameters));  
  problem.AddConstraint(
      std::make_unique<SapFrictionConeConstraint<double>>(3, J34, parameters));        
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 3, J31, J34, parameters));
  problem.AddConstraint(std::make_unique<SapFrictionConeConstraint<double>>(
      0, 3, J31, J34, parameters));        

  PRINT_VAR(problem.num_constraints());
  PRINT_VAR(problem.num_constrained_dofs());            
  EXPECT_EQ(problem.num_constraints(), 5);
  EXPECT_EQ(problem.num_constrained_dofs(), 15);

  // Verify graph invariants:
  // 1. Constraints involving a single clique are represented by a "loop"
  //    connecting the clique with itself (therefore we expect constraint 2 to
  //    connect clique 3 with itself.)  
  // 2. Constraint gropus are lexicographically sorted by clique pair.
  // 3. Participating cliques are numbered in the order they were originally
  //    defined. Therefore we expect participating cliques to be re-numbered as:
  //    0 --> 0, 1 --> 1, 3 --> 2.   
  //const ContactProblemGraph graph = problem.MakeGraph();


  const SapModel<double> model(&problem);
  EXPECT_EQ(model.num_cliques(), 4);
  EXPECT_EQ(model.num_participating_cliques(), 3);
  EXPECT_EQ(model.num_velocities(), num_velocities);
  EXPECT_EQ(model.num_constraints(), 5);
  EXPECT_EQ(model.num_participating_velocities(), 7);
  EXPECT_EQ(model.num_impulses(), 15);

  // Verify participating cliques are numbered according to the order they were
  // defined originally. That is, we expect participating cliques to be
  // re-numbered as: 0 --> 0, 1 --> 1, 3 --> 2.
  const std::vector<int> cliques_permutation_expected{0, 1, -1, 2};
  EXPECT_EQ(model.cliques_permutation().permutation(),
            cliques_permutation_expected);

  // Verify participating velocities permutation.
  // clang-format off
  const std::vector<int> velocities_permutation_expected{
    0,              // clique 0
    1, 2,           // clique 1
    -1, -1, -1,     // clique 2 (does not participate)
    3, 4, 5,  6};   // clique 3
  // clang-format on
  EXPECT_EQ(model.velocities_permutation().permutation(),
            velocities_permutation_expected);

  PRINT_VAR(model.num_cliques());
  PRINT_VAR(model.num_participating_cliques());
  for (const auto& Ac : model.dynamics_matrix()) {
    PRINT_VARn(Ac);
  }

  PRINT_VAR(model.num_participating_cliques());

  // N.B. We know that participating cliques are enumerated according to the
  // order they were enumerated in the original cliques.
  std::vector<MatrixXd> A_permuted_expected;
  A_permuted_expected.push_back(A[0]);
  A_permuted_expected.push_back(A[1]);
  A_permuted_expected.push_back(A[3]);

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

  // Verify v_star.
  // Clique 2 with velocities (4, 5, 6) does not participate and therefore it
  // does not show up in v_start_participating.
  const VectorXd v_star_participating_expected =
      (VectorXd(7) << 1., 2., 3., 7., 8., 9., 10.).finished();
  EXPECT_EQ(model.v_star(), v_star_participating_expected);

  // Verify p_star = A * v (participating DOFs only).
  const VectorXd p_star_participating_expected =
      (VectorXd(7) << 1., 4., 6., 28., 32., 36., 40.).finished();
  EXPECT_EQ(model.p_star(), p_star_participating_expected);      

  // Unit test MultiplyByDynamicsMatrix().
  const VectorXd v = VectorXd::LinSpaced(10, 1.0, 10.0);
  VectorXd v_participating(model.num_participating_velocities());
  model.velocities_permutation().Apply(v, &v_participating);
  PRINT_VARn(v.transpose());
  PRINT_VARn(v_participating.transpose());

  VectorXd p_participating(model.num_participating_velocities());
  model.MultiplyByDynamicsMatrix(v_participating, &p_participating);

  // clang-format off
  const VectorXd p_participating_expected = (VectorXd(7) << 
    1,                           // clique 0. 1 x 1.
    4, 6,                        // clique 1. 1 x (2, 3).
    28, 32, 36, 40).finished();  // clique 3. 4 x (7, 8, 9, 10).
  // clang-format on
  EXPECT_TRUE(CompareMatrices(p_participating, p_participating_expected));
  PRINT_VARn(p_participating.transpose());

  // Verify constraints bundle.
  // clang-format off
  const MatrixXd J_expected = (MatrixXd(15, 7) <<
    J31, J32, Z34,
    J31, Z32, J34,
    J31, Z32, J34,
    Z31, J32, J34,
    Z31, Z32, J34).finished();
  // clang-format on
  PRINT_VARn(J_expected);

  const MatrixXd J = model.constraints_bundle().J().MakeDenseMatrix();
  EXPECT_TRUE(CompareMatrices(J, J_expected));
  
  PRINT_VARn(J);

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