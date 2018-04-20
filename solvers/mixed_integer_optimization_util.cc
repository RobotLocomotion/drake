#include "drake/solvers/mixed_integer_optimization_util.h"

#include "drake/math/gray_code.h"

#include "drake/solvers/integer_optimization_util.h"

namespace drake {
namespace solvers {
void AddLogarithmicSos2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& y) {
  const int num_lambda = lambda.rows();
  for (int i = 0; i < num_lambda; ++i) {
    prog->AddLinearConstraint(lambda(i) >= 0);
    prog->AddLinearConstraint(lambda(i) <= 1);
  }
  prog->AddLinearConstraint(lambda.sum() == 1);
  const int num_interval = num_lambda - 1;
  const int num_binary_vars = CeilLog2(num_interval);
  DRAKE_DEMAND(y.rows() == num_binary_vars);
  const auto gray_codes = math::CalculateReflectedGrayCodes(num_binary_vars);
  DRAKE_ASSERT(y.rows() == num_binary_vars);
  for (int j = 0; j < num_binary_vars; ++j) {
    symbolic::Expression lambda_sum1 = gray_codes(0, j) == 1 ? lambda(0) : 0;
    symbolic::Expression lambda_sum2 = gray_codes(0, j) == 0 ? lambda(0) : 0;
    for (int i = 1; i < num_lambda - 1; ++i) {
      lambda_sum1 +=
          (gray_codes(i - 1, j) == 1 && gray_codes(i, j) == 1) ? lambda(i) : 0;
      lambda_sum2 +=
          (gray_codes(i - 1, j) == 0 && gray_codes(i, j) == 0) ? lambda(i) : 0;
    }
    lambda_sum1 +=
        gray_codes(num_lambda - 2, j) == 1 ? lambda(num_lambda - 1) : 0;
    lambda_sum2 +=
        gray_codes(num_lambda - 2, j) == 0 ? lambda(num_lambda - 1) : 0;
    prog->AddLinearConstraint(lambda_sum1 <= y(j));
    prog->AddLinearConstraint(lambda_sum2 <= 1 - y(j));
  }
}

void AddSos2Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& y) {
  if (lambda.rows() != y.rows() + 1) {
    throw std::runtime_error(
        "The size of y and lambda do not match when adding the SOS2 "
        "constraint.");
  }
  prog->AddLinearConstraint(lambda.sum() == 1);
  prog->AddLinearConstraint(lambda(0) <= y(0) && lambda(0) >= 0);
  for (int i = 1; i < y.rows(); ++i) {
    prog->AddLinearConstraint(lambda(i) <= y(i - 1) + y(i) && lambda(i) >= 0);
  }
  prog->AddLinearConstraint(lambda.tail<1>()(0) <= y.tail<1>()(0));
  prog->AddLinearConstraint(y.sum() == 1);
}

void AddLogarithmicSos1Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::MatrixXi>& codes) {
  const int num_lambda = lambda.rows();
  const int num_digits = CeilLog2(num_lambda);
  DRAKE_DEMAND(codes.rows() == num_lambda && codes.cols() == num_digits);
  DRAKE_DEMAND(y.rows() == num_digits);
  for (int i = 0; i < num_digits; ++i) {
    DRAKE_ASSERT(y(i).get_type() == symbolic::Variable::Type::BINARY);
  }
  for (int i = 0; i < num_lambda; ++i) {
    prog->AddLinearConstraint(lambda(i) >= 0);
  }
  prog->AddLinearConstraint(lambda.sum() == 1);
  for (int j = 0; j < num_digits; ++j) {
    symbolic::Expression lambda_sum1{0};
    symbolic::Expression lambda_sum2{0};
    for (int k = 0; k < num_lambda; ++k) {
      if (codes(k, j) == 1) {
        lambda_sum1 += lambda(k);
      } else if (codes(k, j) == 0) {
        lambda_sum2 += lambda(k);
      } else {
        throw std::runtime_error("The codes entry can be only 0 or 1.");
      }
    }
    prog->AddLinearConstraint(lambda_sum1 <= y(j));
    prog->AddLinearConstraint(lambda_sum2 <= 1 - y(j));
  }
}

void AddBilinearProductMcCormickEnvelopeMultipleChoice(
    MathematicalProgram* prog, const symbolic::Variable& x,
    const symbolic::Variable& y, const symbolic::Expression& w,
    const Eigen::Ref<const Eigen::VectorXd>& phi_x,
    const Eigen::Ref<const Eigen::VectorXd>& phi_y,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& Bx,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& By) {
  const int m = phi_x.rows();
  const int n = phi_y.rows();
  DRAKE_ASSERT(Bx.rows() == m - 1);
  DRAKE_ASSERT(By.rows() == n - 1);
  const auto xij =
      prog->NewContinuousVariables(m - 1, n - 1, x.get_name() + "_xij");
  const auto yij =
      prog->NewContinuousVariables(m - 1, n - 1, y.get_name() + "_yij");
  prog->AddLinearEqualityConstraint(
      xij.cast<symbolic::Expression>().rowwise().sum().sum() - x, 0);
  prog->AddLinearEqualityConstraint(
      yij.cast<symbolic::Expression>().rowwise().sum().sum() - y, 0);

  const auto Bxy = prog->NewContinuousVariables(
      m - 1, n - 1, x.get_name() + "_" + y.get_name() + "_Bxy");
  prog->AddLinearEqualityConstraint(
      Bxy.cast<symbolic::Expression>().rowwise().sum().sum(), 1);
  // The right-hand side of the constraint on w, we will implement
  // w >= w_constraint_rhs(0)
  // w >= w_constraint_rhs(1)
  // w <= w_constraint_rhs(2)
  // w <= w_constraint_rhs(3)
  Vector4<symbolic::Expression> w_constraint_rhs(0, 0, 0, 0);
  for (int i = 0; i < m - 1; ++i) {
    for (int j = 0; j < n - 1; ++j) {
      prog->AddLinearConstraint(xij(i, j) >= phi_x(i) * Bxy(i, j));
      prog->AddLinearConstraint(xij(i, j) <= phi_x(i + 1) * Bxy(i, j));
      prog->AddLinearConstraint(yij(i, j) >= phi_y(j) * Bxy(i, j));
      prog->AddLinearConstraint(yij(i, j) <= phi_y(j + 1) * Bxy(i, j));

      prog->AddConstraint(
          CreateLogicalAndConstraint(+Bx(i), +By(j), +Bxy(i, j)));

      w_constraint_rhs(0) += xij(i, j) * phi_y(j) + phi_x(i) * yij(i, j) -
                             phi_x(i) * phi_y(j) * Bxy(i, j);
      w_constraint_rhs(1) += xij(i, j) * phi_y(j + 1) +
                             phi_x(i + 1) * yij(i, j) -
                             phi_x(i + 1) * phi_y(j + 1) * Bxy(i, j);
      w_constraint_rhs(2) += xij(i, j) * phi_y(j) + phi_x(i + 1) * yij(i, j) -
                             phi_x(i + 1) * phi_y(j) * Bxy(i, j);
      w_constraint_rhs(3) += xij(i, j) * phi_y(j + 1) + phi_x(i) * yij(i, j) -
                             phi_x(i) * phi_y(j + 1) * Bxy(i, j);
    }
  }
  prog->AddLinearConstraint(w >= w_constraint_rhs(0));
  prog->AddLinearConstraint(w >= w_constraint_rhs(1));
  prog->AddLinearConstraint(w <= w_constraint_rhs(2));
  prog->AddLinearConstraint(w <= w_constraint_rhs(3));
}
}  // namespace solvers
}  // namespace drake
