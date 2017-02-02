#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/variable.h"

namespace drake {
namespace solvers {
template <int rows, int cols>
using MatrixDecisionVariable = Eigen::Matrix<Variable, rows, cols>;
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
