#include "drake/systems/trajectory_optimization/contact_implicit_constraints.h"

#include <limits>
#include <memory>

namespace drake {
namespace systems {
void GeneralNonlinearComplementaryConstraint::
    NonlinearComplementaryNonlinearConstraint::DoEval(
        const Eigen::Ref<const Eigen::VectorXd>& x, Eigen::VectorXd& y) const {
  const Eigen::VectorXd z = x.head(z_size_);
  const Eigen::VectorXd alpha = x.block(z_size_, 0, num_complementary_, 1);
  const Eigen::VectorXd beta = x.tail(num_complementary_);
  y.resize(3 * num_complementary_);
  g_double_(z, y.head(num_complementary_));
  y.head(num_complementary_) -= alpha;
  h_double_(z, y.block(num_complementary_, 0, num_complementary_, 1));
  y.block(num_complementary_, 0, num_complementary_, 1) -= beta;
  y.tail(num_complementary_) = (alpha.array() * beta.array()).matrix();
}

void GeneralNonlinearComplementaryConstraint::
    NonlinearComplementaryNonlinearConstraint::DoEval(
        const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd& y) const {
  y.resize(3 * num_complementary_);
  g_autodiff_(x.head(z_size_), y.head(num_complementary_));
  y.head(num_complementary_) -= x.block(z_size_, 0, num_complementary_, 1);
  h_autodiff_(x.head(z_size_),
              y.block(num_complementary_, 0, num_complementary_, 1));
  y.block(num_complementary_, 0, num_complementary_, 1) -=
      x.tail(num_complementary_);
  y.tail(num_complementary_) =
      (x.block(z_size_, 0, num_complementary_, 1).array() *
       x.tail(num_complementary_).array())
          .matrix();
}

std::tuple<solvers::Binding<solvers::BoundingBoxConstraint>,
           solvers::Binding<solvers::LinearConstraint>,
           solvers::Binding<solvers::Constraint>,
           solvers::VectorXDecisionVariable>
GeneralNonlinearComplementaryConstraint::DoAddConstraintToProgram(
    solvers::MathematicalProgram* prog,
    const Eigen::Ref<const solvers::VectorXDecisionVariable>& z) const {
  DRAKE_ASSERT(z_size_ == z.rows());
  auto alpha = prog->NewContinuousVariables(num_complementary_);
  auto beta = prog->NewContinuousVariables(num_complementary_);
  auto nonlinear_constraint{
      std::make_shared<NonlinearComplementaryNonlinearConstraint>(
          g_double_, g_autodiff_, h_double_, h_autodiff_, num_complementary_,
          z_size_, complementary_epsilon_)};
  auto nonlinear_binding =
      prog->AddConstraint(nonlinear_constraint, {z, alpha, beta});
  auto bounding_box_binding = prog->AddBoundingBoxConstraint(
      0, std::numeric_limits<double>::infinity(), {alpha, beta});
  // An empty linear constraint with 0 rows.
  solvers::Binding<solvers::LinearConstraint> linear_binding{
      std::make_shared<solvers::LinearConstraint>(
          Eigen::Matrix<double, 0, Eigen::Dynamic>::Zero(0, z_size_),
          Eigen::Matrix<double, 0, 1>::Zero(),
          Eigen::Matrix<double, 0, 1>::Zero()),
      z};
  solvers::VectorXDecisionVariable new_vars{2 * num_complementary_};
  new_vars << alpha, beta;
  return std::make_tuple(bounding_box_binding, linear_binding,
                         nonlinear_binding, new_vars);
}
}  // namespace systems
}  // namespace drake
