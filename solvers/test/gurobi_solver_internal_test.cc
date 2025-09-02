#include "drake/solvers/gurobi_solver_internal.h"

#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "drake/solvers/gurobi_solver.h"

namespace drake {
namespace solvers {
namespace internal {
const double kInf = std::numeric_limits<double>::infinity();

template <typename C>
void ExpectVectorEqual(const std::vector<C>& vec1, const std::vector<C>& vec2) {
  EXPECT_EQ(vec1.size(), vec2.size());
  for (int i = 0; i < ssize(vec1); ++i) {
    EXPECT_EQ(vec1[i], vec2[i]);
  }
}

GTEST_TEST(AddL2NormCostVariables, Test) {
  MathematicalProgram prog;
  auto x = prog.NewContinuousVariables<3>();
  Eigen::Matrix<double, 3, 2> A0;
  A0 << 1, 2, 3, 4, 5, 6;
  const Eigen::Vector3d b0(1, 4, 7);
  auto cost0 = prog.AddL2NormCost(A0, b0, x.tail<2>());
  Eigen::Matrix2d A1;
  A1 << -1, -2, -3, -4;
  const Eigen::Vector2d b1(-1, -4);
  auto cost1 = prog.AddL2NormCost(A1, b1, x.head<2>());
  std::vector<bool> is_new_variable(prog.num_vars(), false);
  int num_gurobi_vars = prog.num_vars();
  std::vector<std::vector<int>> lorentz_cone_variable_indices;
  std::vector<char> gurobi_var_type(prog.num_vars(), GRB_CONTINUOUS);
  std::vector<double> xlow(prog.num_vars(), -kInf);
  std::vector<double> xupp(prog.num_vars(), kInf);
  AddL2NormCostVariables(prog.l2norm_costs(), &is_new_variable,
                         &num_gurobi_vars, &lorentz_cone_variable_indices,
                         &gurobi_var_type, &xlow, &xupp);
  ExpectVectorEqual(is_new_variable,
                    std::vector<bool>{{false, false, false, true, true, true,
                                       true, true, true, true}});
  EXPECT_EQ(num_gurobi_vars,
            prog.num_vars() + (1 + cost0.evaluator()->get_sparse_A().rows()) +
                (1 + cost1.evaluator()->get_sparse_A().rows()));
  EXPECT_EQ(lorentz_cone_variable_indices.size(), 2);
  ExpectVectorEqual(lorentz_cone_variable_indices[0],
                    std::vector<int>{{3, 4, 5, 6}});
  ExpectVectorEqual(lorentz_cone_variable_indices[1],
                    std::vector<int>{{7, 8, 9}});
  ExpectVectorEqual(gurobi_var_type,
                    std::vector<char>(num_gurobi_vars, GRB_CONTINUOUS));
  ExpectVectorEqual(xlow, std::vector<double>{{-kInf, -kInf, -kInf, 0, -kInf,
                                               -kInf, -kInf, 0, -kInf, -kInf}});
  ExpectVectorEqual(xupp, std::vector<double>(num_gurobi_vars, kInf));
}

GTEST_TEST(AddL2NormCosts, Test) {
  MathematicalProgram prog;
  const auto x = prog.NewContinuousVariables<3>();
  Eigen::Matrix<double, 3, 2> A0;
  A0 << 1, 2, 3, 4, 5, 6;
  const Eigen::Vector3d b0(4, 5, 6);
  const auto cost0 = prog.AddL2NormCost(A0, b0, x.tail<2>());
  Eigen::Matrix<double, 2, 3> A1;
  A1 << -1, -3, -5, -2, -4, -6;
  const Eigen::Vector2d b1(10, 11);
  const auto cost1 = prog.AddL2NormCost(A1, b1, x);
  std::vector<bool> is_new_variable(prog.num_vars(), false);
  int num_gurobi_vars = prog.num_vars();
  std::vector<std::vector<int>> lorentz_cone_variable_indices;
  std::vector<char> gurobi_var_type(prog.num_vars(), GRB_CONTINUOUS);
  std::vector<double> xlow(prog.num_vars(), -kInf);
  std::vector<double> xupp(prog.num_vars(), kInf);
  AddL2NormCostVariables(prog.l2norm_costs(), &is_new_variable,
                         &num_gurobi_vars, &lorentz_cone_variable_indices,
                         &gurobi_var_type, &xlow, &xupp);
  GRBenv* env{nullptr};
  GRBmodel* model{nullptr};
  GRBemptyenv(&env);
  GRBstartenv(env);
  GRBnewmodel(env, &model, "model", num_gurobi_vars, nullptr, &xlow[0],
              &xupp[0], gurobi_var_type.data(), nullptr);
  int num_gurobi_linear_constraints{0};
  int error = AddL2NormCosts(prog, lorentz_cone_variable_indices, model,
                             &num_gurobi_linear_constraints);
  GRBupdatemodel(model);
  EXPECT_EQ(error, 0);
  EXPECT_EQ(num_gurobi_linear_constraints,
            cost0.evaluator()->get_sparse_A().rows() +
                cost1.evaluator()->get_sparse_A().rows());
  double cost_coeff{0};
  GRBgetdblattrelement(model, "Obj", lorentz_cone_variable_indices[0][0],
                       &cost_coeff);
  EXPECT_EQ(cost_coeff, 1);
  GRBgetdblattrelement(model, "Obj", lorentz_cone_variable_indices[1][0],
                       &cost_coeff);
  EXPECT_EQ(cost_coeff, 1);
  int num_q_constrs;
  GRBgetintattr(model, "NumQConstrs", &num_q_constrs);
  EXPECT_EQ(num_q_constrs, 2);
  GRBfreemodel(model);
  GRBfreeenv(env);
}
}  // namespace internal
}  // namespace solvers
}  // namespace drake

int main(int argc, char** argv) {
  // Ensure that we have the Gurobi license for the entire duration of this
  // test, so that we do not have to release and re-acquire the license for
  // every test.
  auto gurobi_license = drake::solvers::GurobiSolver::AcquireLicense();
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
