#pragma once

#include <list>
#include <Eigen/Core>
#include "drake/common/symbolic_variable.h"

namespace drake {
namespace solvers {

/** MatrixIndeterminate<int, int> is used as an alias for
Eigen::Matrix<symbolic::Variable, int, int>. After resolving aliases, a compiler
does not distinguish between the two.*/
template <int rows, int cols>
using MatrixIndeterminate = Eigen::Matrix<symbolic::Variable, rows, cols>;

/** VectorIndeterminate is used as an alias for
Eigen::Matrix<symbolic::Variable, int, 1>. After resolving aliases, a compiler
does not distinguish between the two. @see MatrixIndeterminate*/
template <int rows>
using VectorIndeterminate = MatrixIndeterminate<rows, 1>;

/** MatrixXIndeterminate is used as an alias for
Eigen::Matrix<symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>. After
resolving aliases, a compiler does not distinguish between the two. @see
MatrixIndeterminate*/
using MatrixXIndeterminate =
    MatrixIndeterminate<Eigen::Dynamic, Eigen::Dynamic>;

/** VectorXIndeterminate is used as an alias for
Eigen::Matrix<symbolic::Variable, Eigen::Dynamic, 1>. After resolving aliases, a
compiler does not distinguish between the two. @see MatrixIndeterminate*/
using VectorXIndeterminate = VectorIndeterminate<Eigen::Dynamic>;

using IndeterminatesRefList = std::list<Eigen::Ref<const VectorXIndeterminate>>;

/**
 * Concatenates each element in \p var_list into a single Eigen vector of
 * indeterminates, returns this concatenated vector.
 */
VectorXIndeterminate ConcatenateIndeterminatesRefList(
    const IndeterminatesRefList& var_list);

}  // end namespace solvers
}  // end namespace drake
