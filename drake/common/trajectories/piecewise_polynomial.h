#pragma once

#include <limits>
#include <vector>

#include <Eigen/Core>

#include "drake/common/polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial_base.h"

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
class PiecewisePolynomial : public PiecewisePolynomialBase {
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

  // Computes coeffecients for a cubic spline given the value and first
  // derivatives at the end points.
  // Throws std::runtime_error
  // if \p dt < Eigen::NumTraits<CoefficientType>::epsilon()
  static Eigen::Matrix<CoefficientType, 4, 1> ComputeCubicSplineCoeffs(
      double dt, CoefficientType y0, CoefficientType y1,
      CoefficientType yd0, CoefficientType yd1);

  // Setups the linear constraints for solving coeffs for cubic splines:
  // interior segments have continuous value, first and second derivatives,
  // and end points' values match the given ones.
  // There needs two more constraints to fully solve for the coeffs.
  static int SetupCubicSplineInterieorCoeffsLinearSystem(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y,
      int row, int col,
      Eigen::Matrix<CoefficientType, Eigen::Dynamic, Eigen::Dynamic>* A,
      Eigen::Matrix<CoefficientType, Eigen::Dynamic, 1>* b);

  // Computes the first derivative at the end point using a non-centered,
  // shape-preserving three-point formulae.
  static CoefficientMatrix ComputePchipEndSlope(
      double dt0, double dt1,
      const CoefficientMatrix& slope0,
      const CoefficientMatrix& slope1);

  // Throws std::runtime_error if
  // \p T and \Y have different length,
  // \p T is not strictly increasing,
  // \p Y have inconsistent dimensions,
  // \P T's length is smaller than min_length.
  static void CheckSplineGenerationInputValidityOrThrow(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y,
      int min_length);

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
   * Constructs a const PiecewisePolynomial.
   *
   * @throws std::runtime_error if
   *    \p T and \p Y have different length,
   *    \p T is not strictly increasing,
   *    \p Y have inconsistent dimensions,
   *    \p T has length smaller than 1.
   */
  static PiecewisePolynomial<CoefficientType> ZeroOrderHold(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y);

  /**
   * Constructs a linear PiecewisePolynomial.
   *
   * @throws std::runtime_error if
   *    \p T and \p Y have different length,
   *    \p T is not strictly increasing,
   *    \p Y have inconsistent dimensions,
   *    \p T has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> FirstOrderHold(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y);

  /**
   * Constructs a third order PiecewisePolynomial from \p T and \p Y.
   * First derivatives are chosen to be "shape preserving", i.e. if
   * \p Y is monotonic within some interval, the interpolated data will
   * also be monotonic.
   * The second derivative is not guaranteed to be smooth across the
   * entire spline.
   *
   * The first and last first derivative is chosen using a
   * non-centered, shape-preserving three-point formulae.
   *
   * @throws std::runtime_error if
   *    \p T and \p Y have different length,
   *    \p T is not strictly increasing,
   *    \p Y have inconsistent dimensions,
   *    \p T has length smaller than 3.
   */
  static PiecewisePolynomial<CoefficientType> Pchip(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y);

  /**
   * Constructs a third order PiecewisePolynomial from \p T and \p Y.
   * The PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at \p T.
   * \p Ydot0 and \p Ydot1 are used for the first and last first derivatives.
   *
   * @throws std::runtime_error if
   *    \p T and \p Y have different length,
   *    \p T is not strictly increasing,
   *    \p Y have inconsistent dimensions,
   *    \p Ydot0 or Ydot1 and Y have inconsistent dimensions,
   *    \p T has length smaller than 3.
   */
  static PiecewisePolynomial<CoefficientType> Cubic(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y,
      const CoefficientMatrix& Ydot0,
      const CoefficientMatrix& Ydot1);

  /**
   * Constructs a third order PiecewisePolynomial from \p T, \p Y and \p Ydot.
   * Each segment is fully specified by @Y and @Ydot at both ends.
   * Second derivatives are not continuous.
   *
   * @throws std::runtime_error if
   *    \p T and \p Y have different length,
   *    \p T is not strictly increasing,
   *    \p T and \p Ydot have different length,
   *    \p Y have inconsistent dimensions,
   *    \p Ydot have inconsistent dimensions,
   *    \p T has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> Cubic(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y,
      const std::vector<CoefficientMatrix>& Ydot);

  /**
   * Constructs a third order PiecewisePolynomial from \p T and \p Y.
   * The PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at \p T.
   * "Not-a-knot" end condition is used, which means the third derivatives
   * are continuous for the first two and last two segments.
   *
   * @throws std::runtime_error if
   *    \p T and \p Y have different length,
   *    \p T is not strictly increasing,
   *    \p Y have inconsistent dimensions,
   *    \p T has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> Cubic(
      const std::vector<double>& T,
      const std::vector<CoefficientMatrix>& Y);

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

  int getSegmentPolynomialDegree(int segment_index,
                                 Eigen::Index row = 0,
                                 Eigen::Index col = 0) const override;

  Eigen::Index rows() const override;

  Eigen::Index cols() const override;

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

 protected:
  double segmentValueAtGlobalAbscissa(int segment_index, double t,
                                      Eigen::Index row, Eigen::Index col) const;
};
