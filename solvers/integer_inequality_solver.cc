#include "drake/solvers/integer_inequality_solver.h"

#include <algorithm>
#include <iostream>
#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

namespace drake {
namespace solvers {
namespace integer_programming {

typedef std::vector<std::vector<int>> IntegerList;

template <typename T>
bool IsElementwiseNonnegative(const T & A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (A(i, j) < 0) {
        return false;
      }
    }
  }
  return true;
}

template <typename T>
bool IsElementwiseNonpositive(const T & A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      if (A(i, j) > 0) {
        return false;
      }
    }
  }
  return true;
}

enum ColumnType {Nonnegative, Nonpositive, Indefinite};

template<typename T>
std::vector<ColumnType> ProcessInputs(
    const T & A,
    IntegerList * alphabet) {

  DRAKE_DEMAND(alphabet != NULL);
  int cnt = 0;
  std::vector<ColumnType> ordering(A.cols());

  for (auto & col_alphabet : *alphabet)  {
    ordering.at(cnt) = ColumnType::Indefinite;
    if (IsElementwiseNonnegative<T>(A.col(cnt))) {
      std::sort(col_alphabet.begin(), col_alphabet.end());
      ordering.at(cnt) = ColumnType::Nonnegative;
    } else {
      if (IsElementwiseNonpositive<T>(A.col(cnt))) {
        std::sort(col_alphabet.begin(), col_alphabet.end());
        std::reverse(col_alphabet.begin(), col_alphabet.end());
        ordering.at(cnt) = ColumnType::Nonpositive;
      }
    }
    cnt++;
  }

  return ordering;
}

Eigen::MatrixXi CartesianProduct(
    double point,
    const  Eigen::MatrixXi & tuples) {

  Eigen::MatrixXi cart_product(tuples.rows(), tuples.cols()+1);
  cart_product << Eigen::MatrixXi::Constant(tuples.rows(), 1, point), tuples;

  return cart_product;
}

Eigen::MatrixXi VerticalStack(Eigen::MatrixXi A, Eigen::MatrixXi B) {
  if (A.rows() == 0) return B;

  if (B.rows() == 0) return A;

  Eigen::MatrixXi Y(A.rows()+B.rows(), A.cols());
  Y << A, B;

  return Y;
}

template<typename T>
Eigen::MatrixXi FeasiblePoints(
    const T & A, const T & b,
    const IntegerList & column_alphabets,
    const std::vector<ColumnType> & ordering) {

  Eigen::MatrixXi output;

  for (auto & value : column_alphabets.at(0)) {
    Eigen::MatrixXi new_points;

    if (A.cols() == 1) {
        if (IsElementwiseNonnegative(b-A*value)) {
          new_points.resize(1, 1); new_points(0, 0) = value;
        }
    } else {
        new_points = CartesianProduct(value,
        FeasiblePoints<T>(A.block(0, 1, A.rows(), A.cols()-1), b-A.col(0)*value,
            IntegerList(column_alphabets.begin()+1, column_alphabets.end()),
            std::vector<ColumnType>(ordering.begin()+1, ordering.end())));
    }

    if (new_points.rows() > 0)
      output = VerticalStack(output, new_points);
    else
        if (ordering.at(0) != ColumnType::Indefinite) return output;
  }

  return output;
}

IntegerList BuildAlphabetFromBounds(
    const Eigen::VectorXi & lower_bound,
    const Eigen::VectorXi & upper_bound) {

  DRAKE_DEMAND(lower_bound.size() == upper_bound.size());

  IntegerList alphabet(lower_bound.size()); int cnt = 0;

  for (auto & col_alphabet : alphabet) {
    DRAKE_DEMAND(lower_bound(cnt) <= upper_bound(cnt));

    for (int i = lower_bound(cnt); i <= upper_bound(cnt); i++) {
      col_alphabet.push_back(i);
    }
    cnt++;
  }

  return alphabet;
}

}  // namespace integer_programming

using drake::solvers::integer_programming::BuildAlphabetFromBounds;
using drake::solvers::integer_programming::ProcessInputs;

Eigen::MatrixXi EnumerateIntegerSolutions(
                const Eigen::MatrixXd & A,
                const Eigen::MatrixXd & b,
                const Eigen::VectorXi & lower_bound,
                const Eigen::VectorXi & upper_bound) {
  auto alphabet = BuildAlphabetFromBounds(lower_bound, upper_bound);
  auto ordering = ProcessInputs<Eigen::MatrixXd>(A, &alphabet);

  return drake::solvers::integer_programming::
         FeasiblePoints<Eigen::MatrixXd>(A, b, alphabet, ordering);
}

Eigen::MatrixXi EnumerateIntegerSolutions(
    const Eigen::MatrixXi & A,
    const Eigen::MatrixXi & b,
    const Eigen::VectorXi & lower_bound,
    const Eigen::VectorXi & upper_bound) {

  auto alphabet = BuildAlphabetFromBounds(lower_bound, upper_bound);
  auto ordering = ProcessInputs<Eigen::MatrixXi>(A, &alphabet);

  return drake::solvers::integer_programming::
         FeasiblePoints<Eigen::MatrixXi>(A, b, alphabet, ordering);
}

}  // namespace solvers
}  // namespace drake
