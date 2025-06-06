#include "drake/planning/iris/iris_common.h"

#include "drake/geometry/optimization/hyperellipsoid.h"

namespace drake {
namespace planning {

IrisParameterizationFunction::IrisParameterizationFunction(
    const multibody::RationalForwardKinematics* kin,
    const Eigen::Ref<const Eigen::VectorXd>& q_star_val) {
  const int dimension = kin->plant().num_positions();
  DRAKE_DEMAND(dimension > 0);

  parameterization_ = [kin, q_star_captured = Eigen::VectorXd(q_star_val)](
                          const Eigen::VectorXd& s_val) {
    return kin->ComputeQValue(s_val, q_star_captured);
  };

  parameterization_is_threadsafe_ = true;
  parameterization_dimension_ = dimension;
}

IrisParameterizationFunction::IrisParameterizationFunction(
    const Eigen::VectorX<symbolic::Expression>& expression_parameterization,
    const Eigen::VectorX<symbolic::Variable>& variables) {
  // First, we check that the variables in expression_parameterization match the
  // user-supplied variables.
  symbolic::Variables expression_variables;
  for (const auto& expression : expression_parameterization) {
    expression_variables.insert(expression.GetVariables());
  }
  symbolic::Variables user_supplied_variables(variables);
  DRAKE_THROW_UNLESS(expression_variables == user_supplied_variables);

  // Check for duplicates in variables.
  DRAKE_THROW_UNLESS(variables.size() == ssize(user_supplied_variables));

  int dimension = ssize(expression_variables);

  // Note that in this lambda, we copy the shared_ptr variables, ensuring that
  // variables is kept alive without making a copy of the individual Variable
  // objects (which would break the substitution machinery).
  parameterization_ =
      [expression_parameterization_captured =
           Eigen::VectorX<symbolic::Expression>(expression_parameterization),
       variables_captured = Eigen::VectorX<symbolic::Variable>(variables)](
          const Eigen::VectorXd& q) {
        DRAKE_ASSERT(q.size() == variables_captured.size());
        symbolic::Environment env;
        for (int i = 0; i < q.size(); ++i) {
          env.insert(variables_captured[i], q[i]);
        }
        Eigen::VectorXd out = expression_parameterization_captured.unaryExpr(
            [&env](const symbolic::Expression& expression) {
              return expression.Evaluate(env);
            });
        return out;
      };

  parameterization_is_threadsafe_ = true;
  parameterization_dimension_ = dimension;
}

namespace internal {
int unadaptive_test_samples(double epsilon, double delta, double tau) {
  return static_cast<int>(-2 * std::log(delta) / (tau * tau * epsilon) + 0.5);
}
float calc_delta_min(double delta, int max_iterations) {
  return delta * 6 / (M_PI * M_PI * max_iterations * max_iterations);
}

// Add the tangent to the (scaled) ellipsoid at @p point as a
// constraint.
void AddTangentToPolytope(
    const geometry::optimization::Hyperellipsoid& E,
    const Eigen::Ref<const Eigen::VectorXd>& point,
    double configuration_space_margin,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>* A,
    Eigen::VectorXd* b, int* num_constraints) {
  while (*num_constraints >= A->rows()) {
    // Increase pre-allocated polytope size.
    A->conservativeResize(A->rows() * 2, A->cols());
    b->conservativeResize(b->rows() * 2);
  }

  A->row(*num_constraints) =
      (E.A().transpose() * E.A() * (point - E.center())).normalized();
  (*b)[*num_constraints] =
      A->row(*num_constraints) * point - configuration_space_margin;
  if (A->row(*num_constraints) * E.center() > (*b)[*num_constraints]) {
    throw std::logic_error(
        "The current center of the IRIS region is within "
        "options.sampled_iris_options.configuration_space_margin of being "
        "infeasible.  Check your sample point and/or any additional "
        "constraints you've passed in via the options. The configuration space "
        "surrounding the sample point must have an interior.");
  }
  *num_constraints += 1;
}

}  // namespace internal
}  // namespace planning
}  // namespace drake
