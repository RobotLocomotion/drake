#include "drake/solvers/mixed_integer_optimization_util.h"

#include "drake/math/gray_code.h"
#include "drake/solvers/integer_optimization_util.h"

using drake::symbolic::Expression;

namespace drake {
namespace solvers {
std::string to_string(IntervalBinning binning) {
  switch (binning) {
    case IntervalBinning::kLinear: {
      return "linear_binning";
    }
    case IntervalBinning::kLogarithmic: {
      return "logarithmic_binning";
    }
  }
  DRAKE_UNREACHABLE();
}

std::ostream& operator<<(std::ostream& os, const IntervalBinning& binning) {
  os << to_string(binning);
  return os;
}

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
  prog->AddLinearConstraint(lambda.tail<1>()(0) >= 0 &&
                            lambda.tail<1>()(0) <= y.tail<1>()(0));
  prog->AddLinearConstraint(y.sum() == 1);
}

void AddLogarithmicSos1Constraint(
    MathematicalProgram* prog,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& lambda,
    const Eigen::Ref<const VectorXDecisionVariable>& y,
    const Eigen::Ref<const Eigen::MatrixXi>& binary_encoding) {
  const int num_lambda = lambda.rows();
  const int num_y = CeilLog2(num_lambda);
  DRAKE_DEMAND(binary_encoding.rows() == num_lambda &&
               binary_encoding.cols() == num_y);
  DRAKE_DEMAND(y.rows() == num_y);
  for (int i = 0; i < num_y; ++i) {
    DRAKE_ASSERT(y(i).get_type() == symbolic::Variable::Type::BINARY);
  }
  for (int i = 0; i < num_lambda; ++i) {
    prog->AddLinearConstraint(lambda(i) >= 0);
  }
  prog->AddLinearConstraint(lambda.sum() == 1);
  for (int j = 0; j < num_y; ++j) {
    symbolic::Expression lambda_sum1{0};
    symbolic::Expression lambda_sum2{0};
    for (int k = 0; k < num_lambda; ++k) {
      if (binary_encoding(k, j) == 1) {
        lambda_sum1 += lambda(k);
      } else if (binary_encoding(k, j) == 0) {
        lambda_sum2 += lambda(k);
      } else {
        throw std::runtime_error(
            "The binary_encoding entry can be only 0 or 1.");
      }
    }
    prog->AddLinearConstraint(lambda_sum1 <= y(j));
    prog->AddLinearConstraint(lambda_sum2 <= 1 - y(j));
  }
}

std::pair<VectorX<symbolic::Variable>, VectorX<symbolic::Variable>>
AddLogarithmicSos1Constraint(MathematicalProgram* prog, int num_lambda) {
  const int num_y = CeilLog2(num_lambda);
  const Eigen::MatrixXi binary_encoding =
      math::CalculateReflectedGrayCodes(num_y).topRows(num_lambda);
  auto lambda = prog->NewContinuousVariables(num_lambda);
  auto y = prog->NewBinaryVariables(num_y);
  AddLogarithmicSos1Constraint(prog, lambda.cast<symbolic::Expression>(), y,
                               binary_encoding);
  return std::make_pair(lambda, y);
}

void AddBilinearProductMcCormickEnvelopeMultipleChoice(
    MathematicalProgram* prog, const symbolic::Variable& x,
    const symbolic::Variable& y, const symbolic::Expression& w,
    const Eigen::Ref<const Eigen::VectorXd>& phi_x,
    const Eigen::Ref<const Eigen::VectorXd>& phi_y,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& Bx,
    const Eigen::Ref<const VectorX<symbolic::Expression>>& By) {
  const int m = phi_x.rows() - 1;
  const int n = phi_y.rows() - 1;
  DRAKE_ASSERT(Bx.rows() == m);
  DRAKE_ASSERT(By.rows() == n);
  const auto x_bar = prog->NewContinuousVariables(n, x.get_name() + "_x_bar");
  const auto y_bar = prog->NewContinuousVariables(m, y.get_name() + "_y_bar");
  prog->AddLinearEqualityConstraint(x_bar.cast<Expression>().sum() - x, 0);
  prog->AddLinearEqualityConstraint(y_bar.cast<Expression>().sum() - y, 0);

  const auto Bxy = prog->NewContinuousVariables(
      m, n, x.get_name() + "_" + y.get_name() + "_Bxy");
  prog->AddLinearEqualityConstraint(
      Bxy.cast<Expression>().rowwise().sum().sum(), 1);

  for (int i = 0; i < m; ++i) {
    prog->AddLinearConstraint(
        y_bar(i) >=
        phi_y.head(n).dot(Bxy.row(i).transpose().cast<Expression>()));
    prog->AddLinearConstraint(
        y_bar(i) <=
        phi_y.tail(n).dot(Bxy.row(i).transpose().cast<Expression>()));
  }
  for (int j = 0; j < n; ++j) {
    prog->AddLinearConstraint(x_bar(j) >=
                              phi_x.head(m).dot(Bxy.col(j).cast<Expression>()));
    prog->AddLinearConstraint(x_bar(j) <=
                              phi_x.tail(m).dot(Bxy.col(j).cast<Expression>()));
  }
  // The right-hand side of the constraint on w, we will implement
  // w >= w_constraint_rhs(0)
  // w >= w_constraint_rhs(1)
  // w <= w_constraint_rhs(2)
  // w <= w_constraint_rhs(3)
  Vector4<symbolic::Expression> w_constraint_rhs(0, 0, 0, 0);
  w_constraint_rhs(0) = x_bar.cast<Expression>().dot(phi_y.head(n)) +
                        y_bar.cast<Expression>().dot(phi_x.head(m));
  w_constraint_rhs(1) = x_bar.cast<Expression>().dot(phi_y.tail(n)) +
                        y_bar.cast<Expression>().dot(phi_x.tail(m));
  w_constraint_rhs(2) = x_bar.cast<Expression>().dot(phi_y.head(n)) +
                        y_bar.cast<Expression>().dot(phi_x.tail(m));
  w_constraint_rhs(3) = x_bar.cast<Expression>().dot(phi_y.tail(n)) +
                        y_bar.cast<Expression>().dot(phi_x.head(m));
  for (int i = 0; i < m; ++i) {
    for (int j = 0; j < n; ++j) {
      prog->AddConstraint(
          // +v converts a symbolic variable v to a symbolic expression.
          CreateLogicalAndConstraint(+Bx(i), +By(j), +Bxy(i, j)));

      w_constraint_rhs(0) -= phi_x(i) * phi_y(j) * Bxy(i, j);
      w_constraint_rhs(1) -= phi_x(i + 1) * phi_y(j + 1) * Bxy(i, j);
      w_constraint_rhs(2) -= phi_x(i + 1) * phi_y(j) * Bxy(i, j);
      w_constraint_rhs(3) -= phi_x(i) * phi_y(j + 1) * Bxy(i, j);
    }
  }
  prog->AddLinearConstraint(w >= w_constraint_rhs(0));
  prog->AddLinearConstraint(w >= w_constraint_rhs(1));
  prog->AddLinearConstraint(w <= w_constraint_rhs(2));
  prog->AddLinearConstraint(w <= w_constraint_rhs(3));
}
}  // namespace solvers
}  // namespace drake
