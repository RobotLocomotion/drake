#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/drake_copyable.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/trajectories/piecewise_trajectory.h"

namespace drake {
namespace trajectories {

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
 *   }
 *   else return x;
 * }
 * @endcode
 *
 * PiecewisePolynomials can be added, subtracted, and multiplied.
 * They cannot be divided because Polynomials are not closed
 * under division.
 *
 * @tparam T is a scalar type.
 *
 * Explicit instantiations are provided for:
 *
 * - double
 */
template <typename T>
class PiecewisePolynomial final : public PiecewiseTrajectory<T> {
 public:
  // We are final, so this is okay.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewisePolynomial)

  typedef Polynomial<T> PolynomialType;
  typedef MatrixX<PolynomialType> PolynomialMatrix;
  typedef MatrixX<T> CoefficientMatrix;
  typedef Eigen::Ref<CoefficientMatrix> CoefficientMatrixRef;

  /// Default constructor; just leaves segment_times and polynomials empty.
  PiecewisePolynomial() = default;

  /// Single segment and/or constant value constructor.
  template <typename Derived>
  explicit PiecewisePolynomial(const Eigen::MatrixBase<Derived>& value)
      : PiecewiseTrajectory<T>(std::vector<double>(
            {0.0, std::numeric_limits<double>::infinity()})) {
    polynomials_.push_back(value.template cast<PolynomialType>());
  }

  /// Matrix constructor
  ///
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  PiecewisePolynomial(std::vector<PolynomialMatrix> const& polynomials,
                      std::vector<double> const& breaks);

  /// Scalar constructor
  ///
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  PiecewisePolynomial(std::vector<PolynomialType> const& polynomials,
                      std::vector<double> const& breaks);

  ~PiecewisePolynomial() override = default;

  std::unique_ptr<Trajectory<T>> Clone() const override;

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
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> ZeroOrderHold(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /**
   * Eigen version of ZeroOrderHold(breaks, knots) where each column of knots
   * is used as a knot point, and
   *   knots.cols() == breaks.size().
   *
   * @overload PiecewisePolynomial<T> ZeroOrderHold(breaks, knots)
   */
  static PiecewisePolynomial<T> ZeroOrderHold(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots);

  /**
   * Constructs a piecewise linear PiecewisePolynomial.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `breaks` has length smaller than 2.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> FirstOrderHold(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /**
   * Eigen version of FirstOrderHold(breaks, knots) where each column of knots
   * is used as a knot point, and
   *   knots.cols() == breaks.size().
   *
   * @overload PiecewisePolynomial<T> FirstOrderHold(breaks, knots)
   */
  static PiecewisePolynomial<T> FirstOrderHold(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots);

  /**
   * Constructs a third order PiecewisePolynomial from `breaks` and `knots`.
   * First derivatives are chosen to be "shape preserving", i.e. if
   * `knots` is monotonic within some interval, the interpolated data will
   * also be monotonic. The second derivative is not guaranteed to be smooth
   * across the entire spline.
   *
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
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Pchip(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      bool zero_end_point_derivatives = false);

  /**
   * Eigen version of Pchip(breaks, knots, zero_end_point_derivatives)
   * where each column of knots is used as a knot point, and
   *   knots.cols() == breaks.size().
   *
   * @overload PiecewisePolynomial<T> Pchip(breaks, knots,
   * zero_end_point_derivatives)
   */
  static PiecewisePolynomial<T> Pchip(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
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
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      const CoefficientMatrix& knot_dot_start,
      const CoefficientMatrix& knot_dot_end);

  /**
   * Eigen version of Cubic(breaks, knots, knots_dot_start, knots_dot_end)
   * where each column of knots is used as a knot point, and
   *   knots.cols() == breaks.size().
   *
   * @overload PiecewisePolynomial<T> Cubic(breaks, knots, knots_dot_start,
   * knots_dot_end)
   */
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      const Eigen::Ref<const VectorX<T>>& knots_dot_start,
      const Eigen::Ref<const VectorX<T>>& knots_dot_end);

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
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      const std::vector<CoefficientMatrix>& knots_dot);

  /**
   * Eigen version of Cubic(breaks, knots, knots_dot) where each column of knots
   * and knots_dot are used as the knot point/derivative.
   *   knots.cols() == knots_dot.cols() == breaks.size().
   *
   * @overload PiecewisePolynomial<T> Cubic(breaks, knots, knots_dot)
   */
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      const Eigen::Ref<const MatrixX<T>>& knots_dot);

  /**
   * Constructs a third order PiecewisePolynomial from `breaks` and `knots`.
   * The PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at `breaks`. If
   * `periodic_end_condition` is `false` (default), then the "Not-a-knot" end
   * condition is used here, which means the third derivatives are
   * continuous for the first two and last two segments. If
   * `periodic_end_condition` is `true`, then the first and second derivatives
   * between the end of the last segment and the beginning of the first
   * segment will be continuous. Note that the periodic end condition does
   * not require the first and last knot to be collocated, nor does it add
   * an additional knot to connect the first and last segments. Only first
   * and second derivative continuity is enforced.
   * See https://en.wikipedia.org/wiki/Spline_interpolation,
   * https://www.math.uh.edu/~jingqiu/math4364/spline.pdf, and
   * http://www.maths.lth.se/na/courses/FMN081/FMN081-06/lecture11.pdf
   * for more about cubic splines and their end conditions.
   * The MATLAB files "spline.m" and "csape.m" are also good references.
   *
   * @p breaks and @p knots must have at least 3 elements. The "not-a-knot"
   * condition is ill-defined for two knots, and the "periodic" condition
   * would produce a straight line (use `FirstOrderHold` for this instead).
   *
   * @param periodic_end_condition Determines whether the "not-a-knot"
   * (`false`) or the periodic spline (`true`) end condition is used.
   *
   * @throws std::runtime_error if
   *    `breaks` and `knots` have different length,
   *    `breaks` is not strictly increasing,
   *    `knots` has inconsistent dimensions,
   *    `breaks` has length smaller than 3.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      bool periodic_end_condition = false);

  /**
   * Eigen version of Cubic(breaks, knots) where each column of knots is used
   * as a knot point and  knots.cols() == breaks.size().
   *
   * @overload PiecewisePolynomial<T> Cubic(breaks, knots)
   */
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      bool periodic_end_condition = false);

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
  PiecewisePolynomial<T> derivative(int derivative_order = 1) const;

  std::unique_ptr<Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override {
    return derivative(derivative_order).Clone();
  };

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
  PiecewisePolynomial<T> integral(double value_at_start_time = 0.0) const;

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
  PiecewisePolynomial<T> integral(
      const CoefficientMatrixRef& value_at_start_time) const;

  bool empty() const { return polynomials_.empty(); }

  double scalarValue(double t, Eigen::Index row = 0,
                     Eigen::Index col = 0) const;

  /**
   * Evaluates the PiecewisePolynomial at the given time \p t.
   *
   * @param t The time at which to evaluate the PiecewisePolynomial.
   * @return The matrix of evaluated values.
   */
  MatrixX<T> value(double t) const override;

  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  const PolynomialType& getPolynomial(int segment_index, Eigen::Index row = 0,
                                      Eigen::Index col = 0) const;

  int getSegmentPolynomialDegree(int segment_index, Eigen::Index row = 0,
                                 Eigen::Index col = 0) const;

  /// Returns the row count of each and every PolynomialMatrix segment.
  Eigen::Index rows() const override;

  /// Returns the column count of each and every PolynomialMatrix segment.
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
   * corresponding coefficient of this PiecewisePolynomial.
   * @throws std::exception if any Polynomial in either PiecewisePolynomial is
   * not univariate.
   */
  bool isApprox(const PiecewisePolynomial& other, double tol) const;

  /// Concatenates @p other at the end, yielding a continuous trajectory
  /// from current start_time() to @p other end_time().
  ///
  /// @param other PiecewisePolynomial instance to concatenate.
  /// @throws std::runtime_error if trajectories' dimensions do not match
  ///                            each other (either rows() or cols() does
  ///                            not match between this and @p other).
  /// @throws std::runtime_error if this end_time() and @p other start_time()
  ///                            are not within
  ///                            PiecewiseTrajectory<T>::kEpsilonTime from
  ///                            each other.
  void ConcatenateInTime(const PiecewisePolynomial& other);

  void shiftRight(double offset);

  void setPolynomialMatrixBlock(const PolynomialMatrix& replacement,
                                int segment_number, Eigen::Index row_start = 0,
                                Eigen::Index col_start = 0);

  PiecewisePolynomial slice(int start_segment_index, int num_segments) const;

 private:
  double segmentValueAtGlobalAbscissa(int segment_index, double t,
                                      Eigen::Index row, Eigen::Index col) const;

  static constexpr T kSlopeEpsilon = 1e-10;

  // a PolynomialMatrix for each piece (segment)
  std::vector<PolynomialMatrix> polynomials_;

  // Computes coeffecients for a cubic spline given the value and first
  // derivatives at the end points.
  // Throws std::runtime_error
  // if `dt` < Eigen::NumTraits<T>::epsilon()
  static Eigen::Matrix<T, 4, 1> ComputeCubicSplineCoeffs(double dt, T y0, T y1,
                                                         T yd0, T yd1);

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
      MatrixX<T>* A, VectorX<T>* b);

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

}  // namespace trajectories
}  // namespace drake

