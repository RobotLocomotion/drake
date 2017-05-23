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
using MatrixIndeterminate =
    Eigen::Matrix<symbolic::Variable, rows, cols>;
template <int rows>
using VectorIndeterminate = MatrixIndeterminate<rows, 1>;
using MatrixXIndeterminate =
    MatrixIndeterminate<Eigen::Dynamic, Eigen::Dynamic>;
using VectorXIndeterminate =
    VectorIndeterminate<Eigen::Dynamic>;

using IndeterminatesRefList =
    std::list<Eigen::Ref<const VectorXIndeterminate>>;

/**
 * Concatenates each element in \p var_list into a single Eigen vector of
 * indeterminates, returns this concatenated vector.
 */
VectorXIndeterminate ConcatenateIndeterminatesRefList(
    const IndeterminatesRefList& var_list);
}  // end namespace solvers
}  // end namespace drake
