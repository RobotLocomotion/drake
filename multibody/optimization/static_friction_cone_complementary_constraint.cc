#include "drake/multibody/optimization/static_friction_cone_complementary_constraint.h"

#include <limits>

namespace drake {
namespace multibody {
namespace internal {
const double kInf = std::numeric_limits<double>::infinity();
StaticFrictionConeComplementaryNonlinearConstraint::
    StaticFrictionConeComplementaryNonlinearConstraint(
        const ContactWrenchEvaluator* contact_wrench_evaluator,
        double complementary_tolerance)
    : solvers::Constraint(4,
                          contact_wrench_evaluator->plant().num_positions() +
                              contact_wrench_evaluator->num_lambda() + 2,
                          Eigen::Vector4d::Zero(),
                          Eigen::Vector4d(kInf, 0, 0, complementary_tolerance)),
      contact_wrench_evaluator_{contact_wrench_evaluator},
      alpha_var_{"alpha"},
      beta_var_{"beta"} {}
}  // namespace internal
}  // namespace multibody
}  // namespace drake
