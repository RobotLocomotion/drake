#include "drake/solvers/integer_inequality_solver.h"

#include <algorithm>
#include <functional>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
namespace {

using IntegerVectorList = std::vector<std::vector<int>>;
using SolutionList = Eigen::Matrix<int, -1, -1, Eigen::RowMajor>;

bool IsElementwiseNonnegative(const Eigen::MatrixXi& A) {
  return (A.array() >= 0).all();
}

bool IsElementwiseNonpositive(const Eigen::MatrixXi& A) {
  return (A.array() <= 0).all();
}

/* Construct finite-set of admissible values, i.e., an alphabet, for each
 * coordinate from specified upper and lower bounds, e.g., a lower bound and
 * upper bound of -1, 2 translates to the alphabet [-1, 0, 1, 2]. This function
 * exists only to simplify the external interface.
*/
IntegerVectorList BuildAlphabetFromBounds(const Eigen::VectorXi& lower_bound,
                                          const Eigen::VectorXi& upper_bound) {
  DRAKE_ASSERT(lower_bound.size() == upper_bound.size());

  IntegerVectorList alphabet(lower_bound.size());
  int cnt = 0;

  for (auto& col_alphabet : alphabet) {
    DRAKE_ASSERT(lower_bound(cnt) <= upper_bound(cnt));

    for (int i = lower_bound(cnt); i <= upper_bound(cnt); i++) {
      col_alphabet.push_back(i);
    }
    cnt++;
  }

  return alphabet;
}

/* If a column Ai of A is nonnegative (resp. nonpositive), then  {Ai*z : z ∈ Qi}
 * is totally ordered, where Qi is the alphabet for the iᵗʰ component. In other
 * words, the inequalities Ai z1 ≤ Ai z2 ≤ ... ≤ Ai zm hold for zj ∈ Qi
 * sorted in ascending (resp. descending) order. This allows for infeasibility
 * propagation in the recursive enumeration of integer solutions.  This function
 * detects when {Ai z : z ∈ Qi} is totally ordered and then sorts the alphabet
 * using this ordering.
 */
enum class ColumnType { Nonnegative, Nonpositive, Indefinite };
std::vector<ColumnType> ProcessInputs(const Eigen::MatrixXi& A,
                                      IntegerVectorList* alphabet) {
  DRAKE_ASSERT(alphabet != nullptr);
  int cnt = 0;
  std::vector<ColumnType> column_type(A.cols(), ColumnType::Indefinite);

  for (auto& col_alphabet : *alphabet) {
    if (IsElementwiseNonnegative(A.col(cnt))) {
      std::sort(col_alphabet.begin(), col_alphabet.end());
      column_type[cnt] = ColumnType::Nonnegative;
    } else if (IsElementwiseNonpositive(A.col(cnt))) {
      std::sort(col_alphabet.begin(), col_alphabet.end(), std::greater<int>());
      column_type[cnt] = ColumnType::Nonpositive;
    }
    cnt++;
  }

  return column_type;
}

/* Given an integer z and a list of integer vectors V, constructs
 * the Cartesian product (v, z) for all v ∈ V */
SolutionList CartesianProduct(const SolutionList& V, int z) {
  SolutionList cart_products(V.rows(), V.cols() + 1);
  cart_products << V, SolutionList::Constant(V.rows(), 1, z);

  return cart_products;
}

SolutionList VerticalStack(const SolutionList& A,
                              const SolutionList& B) {
  DRAKE_ASSERT(A.cols() == B.cols());
  if (A.rows() == 0) {
    return B;
  }
  if (B.rows() == 0) {
    return A;
  }
  SolutionList Y(A.rows() + B.rows(), B.cols());
  Y << A, B;
  return Y;
}

/* Find each solution (x(1), x(2), ..., x(n)) to Ax <= b when x(i) can only take
on values in a finite alphabet Qi, e.g., Qi = {1, 2, 3, 8, 9}.   We do this
by recursively enumerating the solutions to
A (x(1), x(2), ..., x(n-1) ) <= (b - A_n x_n) for all values of x(n). If the
column vectors {An*z : z ∈ Qn} are totally ordered, we propagate infeasibility:
if no solutions exist when x(n) = z, then no solution can exist if x(n) takes
on values larger than z (in the ordering).  We assume the function
"ProcessInputs" was previously called to sort the alphabet in ascending order.
*/
// TODO(frankpermenter):  Update to use preallocated memory
SolutionList FeasiblePoints(const Eigen::MatrixXi& A,
                               const Eigen::VectorXi& b,
                               const IntegerVectorList& column_alphabets,
                               const std::vector<ColumnType>& column_type,
                               int last_free_var_pos) {
  DRAKE_ASSERT((last_free_var_pos >= 0) && (last_free_var_pos < A.cols()));
  DRAKE_ASSERT(column_type.size() == static_cast<std::vector<ColumnType>
                                                    ::size_type>(A.cols()));

  SolutionList feasible_points(0, last_free_var_pos + 1);
  for (const auto& value : column_alphabets[last_free_var_pos]) {
    SolutionList new_feasible_points;

    if (last_free_var_pos == 0) {
      if (IsElementwiseNonnegative(b - A.col(0) * value)) {
        new_feasible_points.resize(1, 1);
        new_feasible_points(0, 0) = value;
      }
    } else {
      new_feasible_points = CartesianProduct(
          FeasiblePoints(A,
                        b - A.col(last_free_var_pos) * value,
                        column_alphabets,
                        column_type,
                        last_free_var_pos - 1), value);
    }

    if (new_feasible_points.rows() > 0) {
      feasible_points = VerticalStack(feasible_points, new_feasible_points);
    } else {
      //  Propagate infeasibility: if this test passes, then no feasible
      //  points exist for remaining values in the alphabet.
      if (column_type[last_free_var_pos] != ColumnType::Indefinite) {
        return feasible_points;
      }
    }
  }

  return feasible_points;
}

}  // namespace

SolutionList EnumerateIntegerSolutions(
    const Eigen::Ref<const Eigen::MatrixXi>& A,
    const Eigen::Ref<const Eigen::VectorXi>& b,
    const Eigen::Ref<const Eigen::VectorXi>& lower_bound,
    const Eigen::Ref<const Eigen::VectorXi>& upper_bound) {
  DRAKE_DEMAND(A.rows() == b.rows());
  DRAKE_DEMAND(A.cols() == lower_bound.size());
  DRAKE_DEMAND(A.cols() == upper_bound.size());

  auto variable_alphabets = BuildAlphabetFromBounds(lower_bound, upper_bound);
  // Returns type (nonnegative, nonpositive, or indefinite) of A's
  // columns and sorts the variable alphabet accordingly.
  auto column_type = ProcessInputs(A, &variable_alphabets);

  return FeasiblePoints(A, b, variable_alphabets, column_type, A.cols()-1);
}

}  // namespace solvers
}  // namespace drake
