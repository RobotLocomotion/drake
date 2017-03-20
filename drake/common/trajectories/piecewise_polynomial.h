#pragma once

#include <limits>
#include <vector>

#include <Eigen/Core>

#include "drake/common/eigen_types.h"
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
  typedef drake::MatrixX<PolynomialType> PolynomialMatrix;
  typedef drake::MatrixX<CoefficientType> CoefficientMatrix;
  typedef Eigen::Ref<CoefficientMatrix> CoefficientMatrixRef;

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
                      std::vector<double> const& breaks);

  // Scalar constructor
  PiecewisePolynomial(std::vector<PolynomialType> const& polynomials,
                      std::vector<double> const& breaks);

  /**
   * Constructs a piecewise constant PiecewisePolynomial.
   * Note that constructing a PiecewisePolynomial requires at least two knot
   * points, although in this case, the second knot point's value is ignored,
   * and only its break time is used.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `breaks` has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> ZeroOrderHold(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /**
   * Constructs a piecewise linear PiecewisePolynomial.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `breaks` has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> FirstOrderHold(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /**
   * Constructs a third order PiecewisePolynomial from `breaks` and `knots`.
   * First derivatives are chosen to be "shape preserving", i.e. if
   * `knots` is monotonic within some interval, the interpolated data will
   * also be monotonic.
   * The second derivative is not guaranteed to be smooth across the entire
   * spline.
   * Pchip stands for "Piecewise Cubic Hermite Interpolating Polynomial".
   * For more details, refer to the matlab file "pchip.m".
   * http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf is also a good
   * reference.
   *
   * If @p zero_end_point_derivatives is false, the first and last first
   * derivative is chosen using a non-centered, shape-preserving three-point
   * formulae. See equation (2.10) in the following reference for more details.
   * http://www.mi.sanu.ac.rs/~gvm/radovi/mon.pdf
   * If @p zero_end_point_derivatives is true, they are set to zeros.
   *
   * If @p zero_end_point_derivatives is false, @p breaks and @p knots must
   * have at least 3 elements for the algorithm to determine the first
   * derivatives.
   *
   * If @p zero_end_point_derivatives is true, @p breaks and @p knots may have
   * 2 or more elements. For the 2 elements case, the result is equivalent to
   * computing a cubic polynomial whose values are given by @p knots, and
   * derivatives set to zero.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `breaks` has length smaller than 3 and zero_end_point_derivatives is
   *    false,
   *    `breaks` has length smaller than 2 and zero_end_point_derivatives is
   *    true.
   */
  static PiecewisePolynomial<CoefficientType> Pchip(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      bool zero_end_point_derivatives = false);

  /**
   * Constructs a third order PiecewisePolynomial from `breaks` and `knots`.
   * The PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at `breaks`.
   * `knot_dot_at_start` and `knot_dot_at_end` are used for the first and
   * last first derivatives.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `knots_dot_at_start` or `knot_dot_at_end` and `knots` have
   *    inconsistent dimensions,
   *    `breaks` has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      const CoefficientMatrix& knot_dot_start,
      const CoefficientMatrix& knot_dot_end);

  /**
   * Constructs a third order PiecewisePolynomial from `breaks`, `knots` and
   * `knots`dot.
   * Each segment is fully specified by @knots and @knot_dot at both ends.
   * Second derivatives are not continuous.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `breaks` and `knots`dot have different length,
   *    `knots` has inconsistent dimensions,
   *    `knots_dot` and `knots` have inconsistent dimensions,
   *    `breaks` has length smaller than 2.
   */
  static PiecewisePolynomial<CoefficientType> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      const std::vector<CoefficientMatrix>& knots_dot);

  /**
   * Constructs a third order PiecewisePolynomial from `breaks` and `knots`.
   * The PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at `breaks`.
   * "Not-a-knot" end condition is used here, which means the third derivatives
   * are continuous for the first two and last two segments.
   * See https://en.wikipedia.org/wiki/Spline_interpolation for more details
   * about "Not-a-knot" condition.
   * The matlab file "spline.m" and
   * http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf are also good
   * references.
   *
   * @p breaks and @p knots must have at least 3 elements. Otherwise there is
   * not enough information to solve for the coefficients.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `breaks` has length smaller than 3.
   */
  static PiecewisePolynomial<CoefficientType> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /// Takes the derivative of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial where each segment is the derivative of the
   * segment in the input PiecewisePolynomial.
   * Any rules or limitations of Polynomial::derivative also apply to this
   * function.
   *
   * If `derivative_order` is given, takes the nth derivative of this
   * PiecewisePolynomial.
   */
  PiecewisePolynomial derivative(int derivative_order = 1) const;

  /// Takes the integral of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial that is the indefinite integral of this one.
   * Any rules or limitations of Polynomial::integral also apply to this
   * function.
   *
   * If `value_at_start_time` is given, it does the following only for the
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
   * If `value_at_start_time` is given, it does the following only for the
   * first segment: adds value_at_start_time(row,col) as the constant term
   * (zeroth-order coefficient) of the resulting Polynomial.
   */
  PiecewisePolynomial integral(
      const CoefficientMatrixRef& value_at_start_time) const;

  bool empty() const { return polynomials_.empty(); }

  double scalarValue(double t, Eigen::Index row = 0, Eigen::Index col = 0);

  /**
   * Evaluates the PiecewisePolynomial at the given time \p t.
   *
   * @param t The time at which to evaluate the PiecewisePolynomial.
   * @return The matrix of evaluated values.
   */
  drake::MatrixX<double> value(double t) const;

  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  const PolynomialType& getPolynomial(int segment_index, Eigen::Index row = 0,
                                      Eigen::Index col = 0) const;

  int getSegmentPolynomialDegree(int segment_index, Eigen::Index row = 0,
                                 Eigen::Index col = 0) const override;

  Eigen::Index rows() const override;

  Eigen::Index cols() const override;

  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator-=(const PiecewisePolynomial& other);

  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator+=(const CoefficientMatrix& offset);

  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator-=(const CoefficientMatrix& offset);

  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator+(const PiecewisePolynomial& other) const;

  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator-(const PiecewisePolynomial& other) const;

  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator*(const PiecewisePolynomial& other) const;

  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator+(const CoefficientMatrix& offset) const;

  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator-(const CoefficientMatrix& offset) const;

  /// Checks if a PiecewisePolynomial is approximately equal to this one.
  /**
   * Checks that every coefficient of `other` is within `tol` of the
   * corresponding coefficient of this PiecewisePolynomial. Throws an exception
   * if any Polynomial in either PiecewisePolynomial is not univariate.
   */
  bool isApprox(const PiecewisePolynomial& other, double tol) const;

  void shiftRight(double offset);

  void setPolynomialMatrixBlock(const PolynomialMatrix& replacement,
                                int segment_number, Eigen::Index row_start = 0,
                                Eigen::Index col_start = 0);

  PiecewisePolynomial slice(int start_segment_index, int num_segments) const;

 private:
  double segmentValueAtGlobalAbscissa(int segment_index, double t,
                                      Eigen::Index row, Eigen::Index col) const;

  static constexpr CoefficientType kSlopeEpsilon = 1e-10;

  // a PolynomialMatrix for each piece (segment)
  std::vector<PolynomialMatrix> polynomials_;

  // Computes coeffecients for a cubic spline given the value and first
  // derivatives at the end points.
  // Throws std::runtime_error
  // if `dt` < Eigen::NumTraits<CoefficientType>::epsilon()
  static Eigen::Matrix<CoefficientType, 4, 1> ComputeCubicSplineCoeffs(
      double dt, CoefficientType y0, CoefficientType y1, CoefficientType yd0,
      CoefficientType yd1);

  // For a cubic spline, there are 4 unknowns for each segment Pi, namely
  // the coefficients for Pi = a0 + a1 * t + a2 * t^2 + a3 * t^3.
  // Let N be the size of breaks and knots, there are N-1 segments,
  // and thus 4*(N-1) unknowns to fully specified a cubic spline for the given
  // data.
  //
  // If we are also given N knot_dot (velocity), each Pi will be fully specified
  // by (knots[i], knot_dot[i]) and (knots[i+1], knot_dot[i+1]).
  // When knot_dot are not specified, we make the design choice to enforce
  // continuity up to the second order (Yddot) for the interior points, i.e.
  // Pi'(duration_i) = Pi+1'(0), and Pi''(duration_i) = Pi+1''(0), where
  // ' means time derivative, and duration_i = breaks[i+1] - breaks[i] is the
  // duration for the ith segment.
  //
  // At this point, we have 2 * (N - 1) position constraints:
  // Pi(0) = knots[i], for i in [0, N - 2]
  // Pi(duration_i) = knots[i+1], for i in [0, N - 2]
  // N - 2 velocity constraints for the interior points:
  // Pi'(duration_i) = Pi+1'(0), for i in [0, N - 3]
  // N - 2 acceleration constraints for the interior points:
  // Pi''(duration_i) = Pi+1''(0), for i in [0, N - 3]
  //
  // These sum up to 4 * (N - 1) - 2. This function sets up the above
  // constraints. There are still 2 constraints missing, which can be resolved
  // by various end point conditions (velocity at the end points /
  // "not-a-knot" / etc). These will be specified by the callers.
  static int SetupCubicSplineInteriorCoeffsLinearSystem(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots, int row, int col,
      drake::MatrixX<CoefficientType>* A, drake::VectorX<CoefficientType>* b);

  // Computes the first derivative at the end point using a non-centered,
  // shape-preserving three-point formulae.
  static CoefficientMatrix ComputePchipEndSlope(
      double dt0, double dt1, const CoefficientMatrix& slope0,
      const CoefficientMatrix& slope1);

  // Throws std::runtime_error if
  // `breaks` and `knots` have different length,
  // `breaks` is not strictly increasing,
  // `knots` has inconsistent dimensions,
  // `breaks` has length smaller than min_length.
  static void CheckSplineGenerationInputValidityOrThrow(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots, int min_length);
};
