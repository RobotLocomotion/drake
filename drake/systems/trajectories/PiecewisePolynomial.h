#pragma once

#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_export.h"
#include "drake/common/polynomial.h"
#include "drake/systems/trajectories/PiecewisePolynomialBase.h"

/// A scalar multi-variate piecewise polynomial.
/**
 * PiecewisePolynomial represents a list of contiguous segments in time with a
 * Matrix of Polynomials defined for each segment. The term segment is used for
 * piece.
 *
 * An example of a piecewise polynomial is a function of x segments in time,
 * where for each segment a different polynomial is defined. For a more specific
 * example, consider the absolute value function, which is a piecewise function.
 * It uses one function for inputs values < 0, and another function for input
 * values > 0:
 *
 * @code
 * int abs(int x)
 * {
 *   if (x<0) {
 *     return -x;
     }
 *   else return x;
 * }
 * @endcode
 *
 * PiecewisePolynomials can be added, subtracted, and multiplied.
 * They cannot be divided because Polynomials are not closed
 * under division.
 */
template <typename CoefficientType = double>
class DRAKE_EXPORT PiecewisePolynomial
    : public PiecewisePolynomialBase {
 public:
  typedef Polynomial<CoefficientType> PolynomialType;
  typedef Eigen::Matrix<PolynomialType, Eigen::Dynamic, Eigen::Dynamic>
      PolynomialMatrix;
  typedef Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic>
      CoefficientMatrix;
  typedef Eigen::Ref<CoefficientMatrix> CoefficientMatrixRef;

 private:
  std::vector<PolynomialMatrix>
      polynomials_;  // a PolynomialMatrix for each piece (segment)

 public:
  virtual ~PiecewisePolynomial() {}

  // default constructor; just leaves segment_times and polynomials empty
  PiecewisePolynomial();

  // single segment and/or constant value constructor
  template <typename Derived>
  explicit PiecewisePolynomial(const Eigen::MatrixBase<Derived>& value)
      : PiecewisePolynomialBase(std::vector<double>(
            {{0.0, std::numeric_limits<double>::infinity()}})) {
    polynomials_.push_back(value.template cast<PolynomialType>());
  }

  // Matrix constructor
  PiecewisePolynomial(std::vector<PolynomialMatrix> const& polynomials,
                      std::vector<double> const& segment_times);

  // Scalar constructor
  PiecewisePolynomial(std::vector<PolynomialType> const& polynomials,
                      std::vector<double> const& segment_times);

  /**
   * Construct a PiecewisePolynomial using a first order hold from a
   * series of knot points.
   */
  static PiecewisePolynomial<CoefficientType> FirstOrderHold(
      const std::vector<double>& segment_times,
      const std::vector<CoefficientMatrix>& knots) {
  std::vector<PolynomialMatrix> polys;
  polys.reserve(segment_times.size() - 1);
  for (int i = 0; i < static_cast<int>(segment_times.size()) - 1; ++i) {
    PolynomialMatrix poly_matrix(knots[0].rows(), knots[0].cols());

    for (int j = 0; j < knots[i].rows(); ++j) {
      for (int k = 0; k < knots[i].cols(); ++k) {
        poly_matrix(j, k) = PolynomialType(
            Eigen::Matrix<CoefficientType, 2, 1>(
                knots[i](j, k), (knots[i + 1](j, k) - knots[i](j, k)) /
                (segment_times[i + 1] - segment_times[i])));
      }
    }
    polys.push_back(poly_matrix);
  }
  return PiecewisePolynomial<CoefficientType>(polys, segment_times);
}

  /// Takes the derivative of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial where each segment is the derivative of the
   * segment in the input PiecewisePolynomial.
   * Any rules or limitations of Polynomial::derivative also apply to this
   * function.
   *
   * If \p derivative_order is given, takes the nth derivative of this
   * PiecewisePolynomial.
   */
  PiecewisePolynomial derivative(int derivative_order = 1) const;

  /// Takes the integral of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial that is the indefinite integral of this one.
   * Any rules or limitations of Polynomial::integral also apply to this
   * function.
   *
   * If \p value_at_start_time is given, it does the following only for the
   * first segment: adds that constant as the constant term
   * (zeroth-order coefficient) of the resulting Polynomial.
   */
  PiecewisePolynomial integral(double value_at_start_time = 0.0) const;

  /// Takes the integral of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial that is the indefinite integral of this one.
   * Any rules or limitations of Polynomial::integral also apply to this
   * function.
   *
   * If \p value_at_start_time is given, it does the following only for the
   * first segment: adds value_at_start_time(row,col) as the constant term
   * (zeroth-order coefficient) of the resulting Polynomial.
   */
  PiecewisePolynomial integral(
      const CoefficientMatrixRef& value_at_start_time) const;

  bool empty() const {
    return polynomials_.empty();
  }

  double scalarValue(double t, Eigen::Index row = 0, Eigen::Index col = 0);

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> value(double t) const;

  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  const PolynomialType& getPolynomial(int segment_index, Eigen::Index row = 0,
                                      Eigen::Index col = 0) const;

  virtual int getSegmentPolynomialDegree(int segment_index,
                                         Eigen::Index row = 0,
                                         Eigen::Index col = 0) const;

  virtual Eigen::Index rows() const;

  virtual Eigen::Index cols() const;

  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator-=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator+=(const CoefficientMatrix& offset);

  PiecewisePolynomial& operator-=(const CoefficientMatrix& offset);

  const PiecewisePolynomial operator+(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator-(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator*(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator+(const CoefficientMatrix& offset) const;

  const PiecewisePolynomial operator-(const CoefficientMatrix& offset) const;

  /// Checks if a PiecewisePolynomial is approximately equal to this one.
  /**
   * Checks that every coefficient of \p other is within \p tol of the
   * corresponding coefficient of this PiecewisePolynomial. Throws an exception
   * if any Polynomial in either PiecewisePolynomial is not univariate.
   */
  bool isApprox(const PiecewisePolynomial& other, double tol) const;

  void shiftRight(double offset);

  void setPolynomialMatrixBlock(const PolynomialMatrix& replacement,
                                int segment_number, Eigen::Index row_start = 0,
                                Eigen::Index col_start = 0);

  PiecewisePolynomial slice(int start_segment_index, int num_segments);

  /// Obtains a random PiecewisePolynomial of the specified size.
  /**
   * Obtains a PiecewisePolynomial with the given \p segment_times. Each segment
   * will have a matrix of random Polynomials of the specified size.
   */
  static PiecewisePolynomial random(
      Eigen::Index rows, Eigen::Index cols,
      Eigen::Index num_coefficients_per_polynomial,
      const std::vector<double>& segment_times);

 protected:
  double segmentValueAtGlobalAbscissa(int segment_index, double t,
                                      Eigen::Index row, Eigen::Index col) const;
};
