#pragma once

#include <list>

#include <Eigen/Core>

#include "drake/common/symbolic.h"

namespace drake {
namespace solvers {

/** MatrixIndeterminate<int, int> is used as an alias for
 * Eigen::Matrix<symbolic::Variable, int, int>. After resolving aliases, a
 * compiler does not distinguish between these two. All indeterminates are a
 * variable of type symbolic::Variable::Type::CONTINUOUS (by default).
 * @tparam rows  The number of rows in the new matrix containing indeterminates.
 * @tparam cols  The number of columns in the new matrix containing
 * indeterminates.
 */
template <int rows, int cols>
using MatrixIndeterminate = Eigen::Matrix<symbolic::Variable, rows, cols>;

/** VectorIndeterminate<int> is used as an alias for
 * Eigen::Matrix<symbolic::Variable, int, 1>.
 * @tparam rows  The number of rows in the new matrix containing indeterminates.
 * @see MatrixIndeterminate<int, int>
 */
template <int rows>
using VectorIndeterminate = MatrixIndeterminate<rows, 1>;

/** MatrixXIndeterminate is used as an alias for
 * Eigen::Matrix<symbolic::Variable, Eigen::Dynamic, Eigen::Dynamic>.
 * @see MatrixIndeterminate<int, int>
 */
using MatrixXIndeterminate =
    MatrixIndeterminate<Eigen::Dynamic, Eigen::Dynamic>;

/** VectorXIndeterminate is used as an alias for
 * Eigen::Matrix<symbolic::Variable, Eigen::Dynamic, 1>.
 * @see MatrixIndeterminate<int, int>
 */
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
