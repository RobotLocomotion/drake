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
 * PiecewisePolynomial represents a list of contiguous segments in a scalar
 * independent variable (typically corresponding to time) with Polynomials
 * defined at each segment. We call the output from evaluating the
 * PiecewisePolynomial at the scalar independent variable "the output", and
 * that output can be either a Eigen MatrixX<T> (if evaluated using value())
 * or a scalar (if evaluated using scalar_value()).
 *
 * An example of a piecewise polynomial is a function of m segments in time,
 * where for each segment a different polynomial is defined. For a more specific
 * example, consider the absolute value function over the interval [-1, 1].
 * We can define a PiecewisePolynomial over this interval using breaks at
 * t = { -1, 0, 1 }, and "knots" of abs(t).
 *
 * @code
 * // Construct the PiecewisePolynomial.
 * const std::vector<double> breaks = { -1, 0, 1 };
 * std::vector<Eigen::MatrixXd> knots(3);
 * for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
 *   knots[i].resize(1, 1);
 *   knots[i](0, 0) = std::abs(breaks[i]);
 * }
 * const auto pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
 *
 * // Evaluate the PiecewisePolynomial at some values.
 * std::cout << pp.value(-.5)(0, 0) << std::endl;  // Returns 0.5.
 * std::cout << pp.value(0)(0, 0) << std::endl;    // Returns 0.0;
 *
 * // Show how we can evaluate the first derivative (returns 1.0).
 * std::cout << pp.derivative(1).value(-.5)(0, 0) << std::endl;
 * @endcode
 *
 *
 * PiecewisePolynomials can be added, subtracted, and multiplied.
 * They cannot be divided because Polynomials are not closed
 * under division.
 *
 * @warning PiecewisePolynomials silently clip input evaluations outside of
 * their defined range. So `pp.value(-2)(0, 0)` in the example above would
 * evaluate to -1.0. See value().
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

  /// Single segment, constant value constructor over the interval [0, ∞].
  /// The constructed PiecewisePolynomial will return `constant_value` at
  /// every evaluated point (i.e., `value(t) = constant_value` ∀t ∈ [0, ∞]).
  template <typename Derived>
  explicit PiecewisePolynomial(const Eigen::MatrixBase<Derived>& constant_value)
      : PiecewiseTrajectory<T>(std::vector<double>(
            {0.0, std::numeric_limits<double>::infinity()})) {
    polynomials_.push_back(constant_value.template cast<PolynomialType>());
  }

  /**
   * @anchor polynomial_construction_methods
   * @name Polynomial-based construction methods.
   * Various methods for constructing a PiecewisePolynomial using vectors
   * of matrices of polynomials, one for each output dimension. Unlike the
   * coefficient-based methods, the number of polynomials must equal the number
   * of segments, which will be one fewer than the number of breaks.
   */
  // @{

  /// Constructs a PiecewisePolynomial using matrix-output Polynomials defined
  /// over each segment.
  ///
  /// @pre `polynomials.size() == breaks.size() - 1`
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  PiecewisePolynomial(std::vector<PolynomialMatrix> const& polynomials,
                      std::vector<double> const& breaks);

  /// Constructs a PiecewisePolynomial using scalar-output Polynomials defined
  /// over each segment.
  ///
  /// @pre `polynomials.size() == breaks.size() - 1`
  /// @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
  PiecewisePolynomial(std::vector<PolynomialType> const& polynomials,
                      std::vector<double> const& breaks);
  // @}

  ~PiecewisePolynomial() override = default;

  std::unique_ptr<Trajectory<T>> Clone() const override;

  /**
   * @anchor coefficient_construction_methods
   * @name Coefficient-based construction methods.
   * Various methods for constructing a PiecewisePolynomial using knots of
   * coefficient matrices. Under the hood, PiecewisePolynomial constructs
   * interpolating polynomials that pass through the knot points. These methods
   * differ by the continuity constraints that they enforce at knot points and
   * whether each knot represents a full matrix (versions taking
   * `const std::vector<CoefficientMatrix>&`) or a column vector (versions
   * taking `const Eigen::Ref<const MatrixX<T>>&`).
   *
   * These methods will throw `std::runtime_error` if:
   *  - the breaks and knots have different length,
   *  - the breaks are not strictly increasing,
   *  - the knots have inconsistent dimensions (i.e., the matrices do not all
   *            have identical dimensions),
   *  - the breaks vector has length smaller than 2.
   */
  // @{

  /**
   * Constructs a piecewise constant PiecewisePolynomial using matrix knots.
   * Note that constructing a PiecewisePolynomial requires at least two knot
   * points, although in this case, the second knot point's value is ignored,
   * and only its break time is used.
   *
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> ZeroOrderHold(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /**
   * Version of ZeroOrderHold(breaks, knots) that uses vector knots and
   * Eigen VectorXd/MatrixX<T> inputs. Each column of `knots` represents a knot
   * point.
   *
   * @pre `knots.cols() == breaks.size()`
   * @overload PiecewisePolynomial<T> ZeroOrderHold(breaks, knots)
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> ZeroOrderHold(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots);

  /**
   * Constructs a piecewise linear PiecewisePolynomial using matrix knots.
   *
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> FirstOrderHold(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots);

  /**
   * Version of FirstOrderHold(breaks, knots) that uses vector knots and
   * Eigen VectorXd / MatrixX<T> inputs. Each column of `knots`
   * represents a knot point.
   *
   * @pre `knots.cols() == breaks.size()`
   * @overload PiecewisePolynomial<T> FirstOrderHold(breaks, knots)
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> FirstOrderHold(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots);

  /**
   * Constructs a third order PiecewisePolynomial using matrix knots.
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
   * If `zero_end_point_derivatives` is `false`, the first and last first
   * derivative is chosen using a non-centered, shape-preserving three-point
   * formulae. See equation (2.10) in the following reference for more details.
   * http://www.mi.sanu.ac.rs/~gvm/radovi/mon.pdf
   * If `zero_end_point_derivatives` is `true`, they are set to zeros.
   *
   * If `zero_end_point_derivatives` is `false`, `breaks` and `knots` must
   * have at least 3 elements for the algorithm to determine the first
   * derivatives.
   *
   * If `zero_end_point_derivatives` is `true`, `breaks` and `knots` may have
   * 2 or more elements. For the 2 elements case, the result is equivalent to
   * computing a cubic polynomial whose values are given by `knots`, and
   * derivatives set to zero.
   *
   * @throws std::runtime_error if:
   *  - `breaks` has length smaller than 3 and `zero_end_point_derivatives` is
   *    `false`,
   *  - `breaks` has length smaller than 2 and `zero_end_point_derivatives` is
   *    true.
   *
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Pchip(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      bool zero_end_point_derivatives = false);

  /**
   * Version of Pchip(breaks, knots, zero_end_point_derivatives) that uses
   * vector knots and Eigen VectorXd / MatrixX<T> inputs. Each column of
   * `knots` represents a knot point.
   *
   * @pre `knots.cols() == breaks.size()`.
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @overload PiecewisePolynomial<T> Pchip(breaks, knots,
   * zero_end_point_derivatives)
   */
  static PiecewisePolynomial<T> Pchip(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      bool zero_end_point_derivatives = false);

  /**
   * Constructs a third order PiecewisePolynomial using matrix knots.
   * The PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at `breaks`.
   * `knot_dot_at_start` and `knot_dot_at_end` are used for the first and
   * last first derivatives.
   *
   * @throws std::runtime_error if `knots_dot_at_start` or `knot_dot_at_end`
   * and `knots` have inconsistent dimensions.
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      const CoefficientMatrix& knot_dot_start,
      const CoefficientMatrix& knot_dot_end);

  /**
   * Version of Cubic(breaks, knots, knots_dot_start, knots_dot_end) that uses
   * vector knots and Eigen VectorXd / MatrixX<T> inputs. Each column of `knots`
   * represents a knot point.
   *
   * @pre `knots.cols() == breaks.size()`.
   * @overload PiecewisePolynomial<T> Cubic(breaks, knots, knots_dot_start,
   * knots_dot_end)
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      const Eigen::Ref<const VectorX<T>>& knots_dot_start,
      const Eigen::Ref<const VectorX<T>>& knots_dot_end);

  /**
   * Constructs a third order PiecewisePolynomial using matrix knots and
   * derivatives of knots (`knots_dot`); each matrix element of `knots_dot`
   * represents the derivative with respect to the independent variable (e.g.,
   * the time derivative) of the corresponding entry in `knots`.
   * Each segment is fully specified by `knots` and `knot_dot` at both ends.
   * Second derivatives are not continuous.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      const std::vector<CoefficientMatrix>& knots_dot);

  /**
   * Version of Cubic(breaks, knots, knots_dot) that uses vector knots and
   * Eigen VectorXd / MatrixX<T> inputs. Corresponding columns of `knots` and
   * `knots_dot` are used as the knot point and independent variable derivative,
   * respectively.
   *
   * @overload PiecewisePolynomial<T> Cubic(breaks, knots, knots_dot)
   * @pre `knots.cols() == knots_dot.cols() == breaks.size()`.
   */
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      const Eigen::Ref<const MatrixX<T>>& knots_dot);

  /**
   * Constructs a third order PiecewisePolynomial using matrix knots.
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
   * @param periodic_end_condition Determines whether the "not-a-knot"
   * (`false`) or the periodic spline (`true`) end condition is used.
   *
   * @pre `breaks` and `knots` must have at least 3 elements. The "not-a-knot"
   * condition is ill-defined for two knots, and the "periodic" condition
   * would produce a straight line (use `FirstOrderHold` for this instead).
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> Cubic(
      const std::vector<double>& breaks,
      const std::vector<CoefficientMatrix>& knots,
      bool periodic_end_condition = false);

  /**
   * Version of Cubic(breaks, knots) that uses vector knots and Eigen VectorXd /
   * MatrixX<T> inputs. Each column of `knots` represents a knot point.
   *
   * @pre `knots.cols() == breaks.size()`.
   * @overload PiecewisePolynomial<T> Cubic(breaks, knots)
   */
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const Eigen::VectorXd>& breaks,
      const Eigen::Ref<const MatrixX<T>>& knots,
      bool periodic_end_condition = false);
  // @}

  /// Returns the derivative of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial where each segment is the derivative of the
   * segment in the input PiecewisePolynomial. Any rules or limitations of
   * Polynomial::derivative also apply to this function.
   *
   * Derivatives evaluated at non-differentiable points return the value at the
   * left hand side of the interval.
   *
   * If `derivative_order` is given, takes the nth derivative of this
   * PiecewisePolynomial.
   */
  PiecewisePolynomial<T> derivative(int derivative_order = 1) const;

  std::unique_ptr<Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override {
    return derivative(derivative_order).Clone();
  };

  /// Returns the indefinite integral of this PiecewisePolynomial.
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

  /// Returns the indefinite integral of this PiecewisePolynomial.
  /**
   * Returns a PiecewisePolynomial that is the indefinite integral of this one.
   * Any rules or limitations of Polynomial::integral also apply to this
   * function.
   *
   * If `value_at_start_time` is given, it does the following only for the
   * first segment: adds `value_at_start_time(row,col)` as the constant term
   * (zeroth-order coefficient) of the resulting Polynomial.
   */
  PiecewisePolynomial<T> integral(
      const CoefficientMatrixRef& value_at_start_time) const;

  /// Returns `true` if this trajectory has no breaks/knots/polynomials.
  bool empty() const { return polynomials_.empty(); }

  /// Evaluates the trajectory at the given time without returning the entire
  /// matrix. Equivalent to value(t)(row, col).
  double scalarValue(double t, Eigen::Index row = 0,
                     Eigen::Index col = 0) const;

  /**
   * Evaluates the PiecewisePolynomial at the given time \p t.
   *
   * @param t The time at which to evaluate the PiecewisePolynomial.
   * @return The matrix of evaluated values.
   *
   * @warning If t does not lie in the range that the polynomial is defined
   *          over, the polynomial will silently be evaluated at the closest
   *          point to t. For example, `value(-1)` will return `value(0)` for
   *          a polynomial defined over [0, 1].
   */
  MatrixX<T> value(double t) const override;

  /// Gets the matrix of Polynomials corresponding to the given segment index.
  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  /// Gets the Polynomial with the given matrix row and column index that
  /// corresponds to the given segment index.
  /// Equivalent to `getPolynomialMatrix(segment_index)(row, col)`.
  const PolynomialType& getPolynomial(int segment_index, Eigen::Index row = 0,
                                      Eigen::Index col = 0) const;

  /// Gets the degree of the Polynomial with the given matrix row and column
  /// index that corresponds to the given segment index. Equivalent to
  /// `getPolynomial(segment_index, row, col).GetDegree()`.
  int getSegmentPolynomialDegree(int segment_index, Eigen::Index row = 0,
                                 Eigen::Index col = 0) const;

  /// Returns the row count of the output matrices.
  Eigen::Index rows() const override;

  /// Returns the column count of the output matrices.
  Eigen::Index cols() const override;

  /// Adds all of the values from the knot points in `other` to the knot points
  /// in `this`, storing the result in `this`.
  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  /// Subtracts all of the values of the knot points in `other` from the knot
  /// points in `this`, storing the result in `this`.
  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator-=(const PiecewisePolynomial& other);

  /// Multiplies all of the values of the knot points in `other` by the knot
  /// points in `this` (i.e., a coefficient-wise multiplication), storing the
  /// result in this.
  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  /// Adds all of the values from the knot points in `coeff` to the knot points
  /// in `this`, storing the result in `this`.
  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator+=(const CoefficientMatrix& coeff);

  /// Subtracts all of the values of the knot points in `coeff` from the knot
  /// points in `this`, storing the result in `this`.
  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  PiecewisePolynomial& operator-=(const CoefficientMatrix& coeff);

  /// Adds all of the values from the knot points in `other` to the knot points
  /// in `this`.
  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator+(const PiecewisePolynomial& other) const;

  /// Subtracts all of the values of the knot points in `other` from the knot
  /// points in `this`.
  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator-(const PiecewisePolynomial& other) const;

  /// Multiplies all of the values of the knot points in `other` by the knot
  /// points in `this` (i.e., a coefficient-wise multiplication).
  /// @throws std::runtime_error if other.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator*(const PiecewisePolynomial& other) const;

  /// Adds all of the values from the knot points in `coeff` to the knot points
  /// in `this`.
  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator+(const CoefficientMatrix& coeff) const;

  /// Subtracts all of the values of the knot points in `coeff` from the knot
  /// points in `this`.
  /// @throws std::runtime_error if offset.segment_times is not within
  /// PiecewiseFunction::kEpsilonTime from this->segment_times.
  const PiecewisePolynomial operator-(const CoefficientMatrix& coeff) const;

  /// Checks if a PiecewisePolynomial is approximately equal to this one.
  /**
   * Checks that every coefficient of `other` is within `tol` of the
   * corresponding coefficient of this PiecewisePolynomial.
   * @throws std::exception if any Polynomial in either PiecewisePolynomial is
   * not univariate.
   */
  bool isApprox(const PiecewisePolynomial& other, double tol) const;

  /// Concatenates `other` at the end, yielding a continuous trajectory
  /// from current start_time() to `other` end_time().
  ///
  /// @param other PiecewisePolynomial instance to concatenate.
  /// @throws std::runtime_error if trajectories' dimensions do not match
  ///                            each other (either rows() or cols() does
  ///                            not match between this and `other`).
  /// @throws std::runtime_error if this end_time() and `other` start_time()
  ///                            are not within
  ///                            PiecewiseTrajectory<T>::kEpsilonTime from
  ///                            each other.
  void ConcatenateInTime(const PiecewisePolynomial& other);

  /// Adds `offset` to all of the breaks. `offset` need not be a non-negative
  /// number.
  void shiftRight(double offset);

  /// Replaces the specified block of the PolynomialMatrix at the given
  /// segment index.
  void setPolynomialMatrixBlock(const PolynomialMatrix& replacement,
                                int segment_index, Eigen::Index row_start = 0,
                                Eigen::Index col_start = 0);

  /// Returns the PiecewisePolynomial comprising the `num_segments` segments
  /// starting at the specified `start_segment_index`.
  PiecewisePolynomial slice(int start_segment_index, int num_segments) const;

 private:
  double segmentValueAtGlobalAbscissa(int segment_index, double t,
                                      Eigen::Index row, Eigen::Index col) const;

  static constexpr T kSlopeEpsilon = 1e-10;

  // a PolynomialMatrix for each piece (segment).
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

