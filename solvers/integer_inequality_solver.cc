#include "drake/solvers/integer_inequality_solver.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
namespace integer_programming {

using IntegerVectorList = std::vector<std::vector<int>>;

bool IsElementwiseNonnegative(const Eigen::MatrixXi & A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (A(i, j) < 0) {
        return false;
      }
    }
  }
  return true;
}

bool IsElementwiseNonpositive(const Eigen::MatrixXi & A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (A(i, j) > 0) {
        return false;
      }
    }
  }
  return true;
}


/* Construct finite-set of admissible values, i.e., an alphabet, for each
 * coordinate from specified upper and lower bounds, e.g., a lower-bound and
 * upperbound of -1,2 translates to the alphabet [-1,0,1,2]. This function
 * exists only to simplify the external interface.
*/
IntegerVectorList BuildAlphabetFromBounds(const Eigen::VectorXi& lower_bound,
                                          const Eigen::VectorXi& upper_bound) {
  DRAKE_DEMAND(lower_bound.size() == upper_bound.size());

  IntegerVectorList alphabet(lower_bound.size());
  int cnt = 0;

  for (auto& col_alphabet : alphabet) {
    DRAKE_DEMAND(lower_bound(cnt) <= upper_bound(cnt));

    for (int i = lower_bound(cnt); i <= upper_bound(cnt); i++) {
      col_alphabet.push_back(i);
    }
    cnt++;
  }

  return alphabet;
}


/* If a column Ai of A is nonnegative (resp. nonpositive),
 * then  \{ Ai z : z \in Qi } is totally ordered, where Qi is the alphabet
 * for the i^th component.  This allows for infeasibility propagation
 * in the recursive enumeration of integer solutions.  This function
 * detects when \{ Ai z : z \in Qi } is totally ordered and then sorts the
 * alphabet using this ordering.
 */
enum ColumnType { Nonnegative, Nonpositive, Indefinite };
std::vector<ColumnType> ProcessInputs(const Eigen::MatrixXi & A, IntegerVectorList* alphabet) {
  DRAKE_DEMAND(alphabet != NULL);
  int cnt = 0;
  std::vector<ColumnType> ordering(A.cols());

  for (auto& col_alphabet : *alphabet) {
    ordering.at(cnt) = ColumnType::Indefinite;
    if (IsElementwiseNonnegative(A.col(cnt))) {
      std::sort(col_alphabet.begin(), col_alphabet.end());
      ordering.at(cnt) = ColumnType::Nonnegative;
    } else if (IsElementwiseNonpositive(A.col(cnt))) {
      std::sort(col_alphabet.begin(), col_alphabet.end());
      std::reverse(col_alphabet.begin(), col_alphabet.end());
      ordering.at(cnt) = ColumnType::Nonpositive;
    }
    cnt++;
  }

  return ordering;
}

/* Given an integer z and a list of integer vectors V, constructs
 * the Cartesian product (z, v) for all v \in V.*/
Eigen::MatrixXi CartesianProduct(int z, const Eigen::MatrixXi& V) {
  Eigen::MatrixXi cart_products(V.rows(), V.cols() + 1);
  cart_products << Eigen::MatrixXi::Constant(V.rows(), 1, z), V;

  return cart_products;
}

Eigen::MatrixXi VerticalStack(const Eigen::MatrixXi& A,
                              const Eigen::MatrixXi& B) {
  if (A.rows() == 0) return B;

  if (B.rows() == 0) return A;

  Eigen::MatrixXi Y(A.rows() + B.rows(), A.cols());
  Y << A, B;

  return Y;
}

/*Find each solution (x1, x2, ..., xn) to Ax <= b when
 * xi can only take on values in a finite alphabet Qi, e.g.,
 * Qi = {1, 2, 3, 8, 9}.   We do this recursively, enumerating
 * the possible values (x2, x3, ..., xn) can take when x1
 * is fixed. If the columns \{Ai z : z \in Qi \} are totally
 * ordered, we propagate infeasibility: if no solutions exist when
 * x1 = z, then no solution can exist if x1 takes on values
 * larger than z (in the ordering).  We assume the function "ProcessInputs"
 * was previously called to sort the alphabet in ascending order.)
 */
Eigen::MatrixXi FeasiblePoints(const Eigen::MatrixXi & A, const Eigen::VectorXi & b,
                               const IntegerVectorList& column_alphabets,
                               const std::vector<ColumnType>& ordering) {
  
  Eigen::MatrixXi feasible_points;

  for (auto& value : column_alphabets.at(0)) {
    Eigen::MatrixXi new_feasible_points;

    if (A.cols() == 1) {
      if (IsElementwiseNonnegative(b - A * value)) {
        new_feasible_points.resize(1, 1);
        new_feasible_points(0, 0) = value;
      }
    } else {
      new_feasible_points = CartesianProduct(
          value,
          FeasiblePoints(
              A.block(0, 1, A.rows(), A.cols() - 1), b - A.col(0) * value,
              IntegerVectorList(column_alphabets.begin() + 1,
                                column_alphabets.end()),
              std::vector<ColumnType>(ordering.begin() + 1, ordering.end())));
    }

    if (new_feasible_points.rows() > 0)
      feasible_points = VerticalStack(feasible_points, new_feasible_points);
    else
        //  Propagate infeasibility: if this test passes, then no feasible
        //  points exist for remaining values in the alphabet.
        if (ordering.at(0) != ColumnType::Indefinite)
      return feasible_points;
  }

  return feasible_points;
}


}  // namespace integer_programming

using drake::solvers::integer_programming::BuildAlphabetFromBounds;
using drake::solvers::integer_programming::ProcessInputs;

Eigen::MatrixXi EnumerateIntegerSolutions(const Eigen::MatrixXi& A,
                                          const Eigen::VectorXi& b,
                                          const Eigen::VectorXi& lower_bound,
                                          const Eigen::VectorXi& upper_bound) {

  DRAKE_DEMAND(A.rows() == b.rows());
  DRAKE_DEMAND(A.cols() == lower_bound.size());
  DRAKE_DEMAND(A.cols() == upper_bound.size());


  auto variable_alphabets = BuildAlphabetFromBounds(lower_bound, upper_bound);
  // Returns type (nonnegative, nonpositive, or indefinite) of A's
  // columns and sorts the variable alphabet accordingly.
  auto column_type = ProcessInputs(A, &variable_alphabets);

  return drake::solvers::integer_programming::FeasiblePoints(
      A, b, variable_alphabets, column_type);
}

}  // namespace solvers
}  // namespace drake
