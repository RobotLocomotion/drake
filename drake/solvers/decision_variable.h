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
using DecisionVariableMatrix =
    Eigen::Matrix<drake::symbolic::Variable, rows, cols>;
template <int rows>
using DecisionVariableVector = DecisionVariableMatrix<rows, 1>;
using DecisionVariableMatrixX =
    DecisionVariableMatrix<Eigen::Dynamic, Eigen::Dynamic>;
using DecisionVariableVectorX = DecisionVariableVector<Eigen::Dynamic>;

using VariableListRef = std::list<Eigen::Ref<const DecisionVariableVectorX>>;

}  // end namespace solvers
}  // end namespace drake
