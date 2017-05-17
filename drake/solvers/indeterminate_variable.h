#pragma once

#include <cstddef>
#include <list>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "drake/common/symbolic_variable.h"

namespace drake {
namespace solvers {
template <int rows, int cols>
using MatrixIndeterminateVariable = Eigen::Matrix<symbolic::Variable, rows, cols>;
template <int rows>
using VectorIndeterminateVariable = MatrixIndeterminateVariable<rows, 1>;
using MatrixXIndeterminateVariable =
    MatrixIndeterminateVariable<Eigen::Dynamic, Eigen::Dynamic>;
using VectorXIndeterminateVariable = VectorIndeterminateVariable<Eigen::Dynamic>;

using IndeterminateVariableRefList = std::list<Eigen::Ref<const VectorXIndeterminateVariable>>;

/**
 * Concatenates each element in \p var_list into a single Eigen vector of
 * indeterminates, returns this concatenated vector.
 */
VectorXIndeterminateVariable ConcatenateIndeterminateVariableRefList(
    const IndeterminateVariableRefList& var_list);
}  // end namespace solvers
}  // end namespace drake
