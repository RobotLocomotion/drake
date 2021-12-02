#include "drake/multibody/contact_solvers/sap_solver.h"

#include <gtest/gtest.h>

#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/multibody/contact_solvers/block_sparse_matrix.h"
#include "drake/multibody/contact_solvers/contact_solver_results.h"
#include "drake/multibody/contact_solvers/contact_solver_utils.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace drake {
namespace multibody {
namespace contact_solvers {
namespace internal {

class SapSolverTester {
 public:
  using PreProcessedData = SapSolver<double>::PreProcessedData;

  static void PackContactResults(const PreProcessedData& data,
                                 const VectorXd& v, const VectorXd& vc,
                                 const VectorXd& gamma,
                                 ContactSolverResults<double>* result) {
    SapSolver<double>::PackContactResults(data, v, vc, gamma, result);
  }
};

GTEST_TEST(SapSolver, PackContactResults) {
  // Setup minimum problem data.
  const int nv = 6;
  const int nc = 3;
  SapSolverTester::PreProcessedData data(1.0e-3, nv, nc);
  MatrixXd Jdense = MatrixXd::Ones(3 * nc, nv);
  BlockSparseMatrixBuilder<double> builder(1, 1, 1);
  builder.PushBlock(0, 0, Jdense);
  data.J = builder.Build();

  // Arbitrary solution vectors.
  const VectorXd v = VectorXd::LinSpaced(nv, 0, nv - 1);
  const VectorXd vc = VectorXd::LinSpaced(3 * nc, 0, 3 * nc - 1);
  const VectorXd gamma = -VectorXd::LinSpaced(3 * nc, 0, 3 * nc - 1);
  ContactSolverResults<double> results;
  SapSolverTester::PackContactResults(data, v, vc, gamma, &results);

  // Verify results were properly packed.
  EXPECT_EQ(results.v_next, v);
  const VectorXd fc_expected = gamma / data.time_step;
  VectorXd results_fc(3 * nc);
  MergeNormalAndTangent(results.fn, results.ft, &results_fc);
  EXPECT_EQ(results_fc, fc_expected);
  VectorXd results_vc(3 * nc);
  MergeNormalAndTangent(results.vn, results.vt, &results_vc);
  EXPECT_EQ(results_vc, vc);
  const VectorXd tauc_expected = Jdense.transpose() * gamma / data.time_step;
  EXPECT_TRUE(CompareMatrices(results.tau_contact, tauc_expected,
                              std::numeric_limits<double>::epsilon()));
}

}  // namespace internal
}  // namespace contact_solvers
}  // namespace multibody
}  // namespace drake
