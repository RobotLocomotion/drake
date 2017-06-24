#pragma once

#include "drake/common/symbolic_expression.h"
#include "drake/common/symbolic_polynomial.h"
#include "drake/solvers/decision_variable.h"

namespace drake {
namespace solvers {
symbolic::Expression ReplaceBilinearTerms(
    const symbolic::Expression& e,
    const Eigen::Ref<const VectorXDecisionVariable>& x,
const Eigen::Ref<const VectorXDecisionVariable>& y,
const Eigen::Ref<const MatrixXDecisionVariable>& W);
}  // namespace solvers
}  // namespace drake