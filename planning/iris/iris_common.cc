#include "drake/planning/iris/iris_common.h"

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

}  // namespace planning
}  // namespace drake
