#pragma once

#include <list>

#include <Eigen/Core>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {

using DecisionVariable = symbolic::Variable;

template <int rows, int cols>
using MatrixDecisionVariable = Eigen::Matrix<symbolic::Variable, rows, cols>;
template <int rows>
using VectorDecisionVariable = MatrixDecisionVariable<rows, 1>;
using MatrixXDecisionVariable =
    MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic>;
using VectorXDecisionVariable = VectorDecisionVariable<Eigen::Dynamic>;

using VariableRefList = std::list<Eigen::Ref<const VectorXDecisionVariable>>;

/**
 * Concatenates each element in \p var_list into a single Eigen vector of
 * decision variables, returns this concatenated vector.
 */
VectorXDecisionVariable ConcatenateVariableRefList(
    const VariableRefList& var_list);
}  // end namespace solvers
}  // end namespace drake
