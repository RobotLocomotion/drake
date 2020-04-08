#pragma once

#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Core>

#include "drake/common/default_scalars.h"
#include "drake/common/drake_copyable.h"
#include "drake/common/drake_deprecated.h"
#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"
#include "drake/common/trajectories/piecewise_trajectory.h"

namespace drake {
namespace trajectories {

/**
 * A scalar multi-variate piecewise polynomial.
 *
 * %PiecewisePolynomial represents a list of contiguous segments in a scalar
 * independent variable (typically corresponding to time) with Polynomials
 * defined at each segment. We call the output from evaluating the
 * %PiecewisePolynomial at the scalar independent variable "the output", and
 * that output can be either a Eigen MatrixX<T> (if evaluated using value())
 * or a scalar (if evaluated using scalar_value()).
 *
 * An example of a piecewise polynomial is a function of m segments in time,
 * where a different polynomial is defined for each segment. For a specific
 * example, consider the absolute value function over the interval [-1, 1].
 * We can define a %PiecewisePolynomial over this interval using breaks at
 * t = { -1.0, 0.0, 1.0 }, and "samples" of abs(t).
 *
 * @code
 * // Construct the PiecewisePolynomial.
 * const std::vector<double> breaks = { -1.0, 0.0, 1.0 };
 * std::vector<Eigen::MatrixXd> samples(3);
 * for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
 *   samples[i].resize(1, 1);
 *   samples[i](0, 0) = std::abs(breaks[i]);
 * }
 * const auto pp =
 *      PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
 * const int row = 0, col = 0;
 *
 * // Evaluate the PiecewisePolynomial at some values.
 * std::cout << pp.value(-.5)(row, col) << std::endl;    // Outputs 0.5.
 * std::cout << pp.value(0.0)(row, col) << std::endl;    // Outputs 0.0;
 *
 * // Show how we can evaluate the first derivative (outputs -1.0).
 * std::cout << pp.derivative(1).value(-.5)(row, col) << std::endl;
 * @endcode
 *
 * A note on terminology.  For piecewise-polynomial interpolation, we use
 * `breaks` to indicate the scalar (e.g. times) which form the boundary of
 * each segment.  We use `samples` to indicate the function value at the
 * `breaks`, e.g. `p(breaks[i]) = samples[i]`.  The term `knot` should be
 * reserved for the "(x,y)" coordinate, here
 * `knot[i] = (breaks[i], samples[i])`, though it is used inconsistently in
 * the interpolation literature (sometimes for `breaks`, sometimes for
 * `samples`), so we try to mostly avoid it here.
 *
 * PiecewisePolynomial objects can be added, subtracted, and multiplied.
 * They cannot be divided because Polynomials are not closed
 * under division.
 *
 * @warning %PiecewisePolynomial silently clips input evaluations outside of
 * the range defined by the breaks. So `pp.value(-2.0, row, col)` in the example
 * above would evaluate to -1.0. See value().
 *
 * @tparam_default_scalars
 */
template <typename T>
class PiecewisePolynomial final : public PiecewiseTrajectory<T> {
 public:
  /**
   *  Constructs an empty piecewise polynomial.
   */
  PiecewisePolynomial() = default;

  // We are final, so this is okay.
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(PiecewisePolynomial)

  DRAKE_DEPRECATED("2020-08-01", "Use Polynomial<T> instead of PolynomialType.")
  typedef Polynomial<T> PolynomialType;

  typedef MatrixX<Polynomial<T>> PolynomialMatrix;

  /**
   * Single segment, constant value constructor over the interval [0, ∞].
   * The constructed %PiecewisePolynomial will return `constant_value` at
   * every evaluated point (i.e., `value(t) = constant_value` ∀t ∈ [0, ∞]).
   */
  template <typename Derived>
  explicit PiecewisePolynomial(const Eigen::MatrixBase<Derived>& constant_value)
      : PiecewiseTrajectory<T>(std::vector<T>(
            {0.0, std::numeric_limits<double>::infinity()})) {
    polynomials_.push_back(constant_value.template cast<Polynomial<T>>());
  }

  /**
   * @anchor polynomial_construction_methods
   * @name Polynomial-based construction methods.
   * Various methods for constructing a %PiecewisePolynomial using vectors
   * of matrices of polynomials, one for each output dimension. Unlike the
   * coefficient-based methods, the number of polynomials must equal the number
   * of segments, which will be one fewer than the number of breaks.
   *
   * The following shows how such a %PiecewisePolynomial might be constructed
   * and used:
   * @code
   * // Construct the PiecewisePolynomial.
   * const std::vector<double> breaks = { -1.0, 0.0, 1.0 };
   * Polynomiald t("t");
   * std::vector<Polynomiald> polynomials = { -(t*t), (t*t) };
   * const PiecewisePolynomial<double> pp(polynomials, breaks);
   *
   * // Evaluate the PiecewisePolynomial at some values.
   * std::cout << pp.scalar_value(-1.0) << std::endl;    // Outputs -1.0
   * std::cout << pp.scalar_value(1.0) << std::endl;     // Outputs 1.0
   * @endcode
   *
   * @anchor polynomial_warning
   * <b>WARNING:</b> For robust floating point arithmetic, the polynomial for
   * a segment will be evaluated (using value()) by first
   * subtracting the break time from the evaluation time. In other words, when t
   * lies in the half-open interval `[breaks[i], breaks[i+1])` then:
   * @code
   * value(t) == polynomials[i].eval(t - breaks[i])
   * @endcode
   * meaning that constructing the polynomial like:
   * @code
   * const std::vector<double> breaks = { 0.0, 1.0, 2.0 };
   * Polynomiald t("t");
   * std::vector<Polynomiald> polynomials = { (t*t), (t*t) };
   * const PiecewisePolynomial<double> pp(polynomials, breaks);
   * @endcode
   * would give the following result:
   * @code
   * // Evaluate the PiecewisePolynomial on both sides of a break.
   * const int row = 0, col = 0;
   * const double eps = 0.5 * std::numeric_limits<double>::epsilon();
   * std::cout << pp.value(1.0-eps)(row, col) << std::endl;    // Outputs 1.0
   * std::cout << pp.value(1.0+eps)(row, col) << std::endl;    // Outputs 1e-32
   * @endcode
   * because the second polynomial will be evaluated at 1.0+eps minus the break
   * time for that polynomial (1.0), i.e., t=eps.
   * The intended result for the above example can be obtained by shifting the
   * piecewise polynomial like so:
   * @code
   * const std::vector<double> breaks = { 0.0, 1.0, 2.0 };
   * Polynomiald t("t");
   * std::vector<Polynomiald> polynomials = { (t*t),
   *     ((t+breaks[1])*(t+breaks[1])) };
   * const PiecewisePolynomial<double> pp(polynomials, breaks);
   *
   * // Evaluate the PiecewisePolynomial on both sides of a break.
   * const double eps = 0.5 * std::numeric_limits<double>::epsilon();
   * std::cout << pp.value(1.0-eps)(row, col) << std::endl;    // Outputs 1.0
   * std::cout << pp.value(1.0+eps)(row, col) << std::endl;    // Outputs 1.0
   * @endcode
   */
  // @{

  /**
   * Constructs a %PiecewisePolynomial using matrix-output Polynomials defined
   * over each segment.
   *
   * @pre `polynomials.size() == breaks.size() - 1`
   */
  PiecewisePolynomial(const std::vector<PolynomialMatrix>& polynomials_matrix,
                      const std::vector<T>& breaks);

  /**
   * Constructs a %PiecewisePolynomial using scalar-output Polynomials defined
   * over each segment.
   *
   * @pre `polynomials.size() == breaks.size() - 1`
   */
  PiecewisePolynomial(const std::vector<Polynomial<T>>& polynomials,
                      const std::vector<T>& breaks);
  // @}

  ~PiecewisePolynomial() override = default;

  std::unique_ptr<Trajectory<T>> Clone() const override;

  /**
   * @anchor coefficient_construction_methods
   * @name Coefficient-based construction methods.
   * Various methods for constructing a %PiecewisePolynomial using samples of
   * coefficient matrices. Under the hood, %PiecewisePolynomial constructs
   * interpolating Polynomial objects that pass through the sample points. These
   * methods differ by the continuity constraints that they enforce at break
   * points and whether each sample represents a full matrix (versions taking
   * `const std::vector<MatrixX<T>>&`) or a column vector (versions
   * taking `const Eigen::Ref<const MatrixX<T>>&`).
   *
   * These methods will throw `std::runtime_error` if:
   *  - the breaks and samples have different length,
   *  - the breaks are not strictly increasing,
   *  - the samples have inconsistent dimensions (i.e., the matrices do not all
   *            have identical dimensions),
   *  - the breaks vector has length smaller than 2.
   */
  // @{

  /**
   * Constructs a piecewise constant %PiecewisePolynomial using matrix samples.
   * Note that constructing a %PiecewisePolynomial requires at least two sample
   * points, although in this case, the second sample point's value is ignored,
   * and only its break time is used.
   *
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> ZeroOrderHold(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples);

  /**
   * Version of ZeroOrderHold(breaks, samples) that uses vector samples and
   * Eigen VectorXd/MatrixX<T> inputs. Each column of `samples` represents a sample
   * point.
   *
   * @pre `samples.cols() == breaks.size()`
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> ZeroOrderHold(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples);

  /**
   * Constructs a piecewise linear %PiecewisePolynomial using matrix samples.
   *
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> FirstOrderHold(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples);

  /**
   * Version of FirstOrderHold(breaks, samples) that uses vector samples and
   * Eigen VectorXd / MatrixX<T> inputs. Each column of `samples`
   * represents a sample point.
   *
   * @pre `samples.cols() == breaks.size()`
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> FirstOrderHold(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples);

  // TODO(russt): This version of the method is not exposed in pydrake, but
  //  the version that is has limited documentation that refers back to this
  //  verbose version.  Either add support for this in pydrake, or flip the
  //  documentation so that pydrake gets the verbose/stand-along version.
  /**
   * Constructs a third order %PiecewisePolynomial using vector samples,
   * where each column of `samples` represents a sample point. First derivatives
   * are chosen to be "shape preserving", i.e. if `samples` is monotonic
   * within some interval, the interpolated data will also be monotonic. The
   * second derivative is not guaranteed to be smooth across the entire spline.
   *
   * MATLAB calls this method "pchip" (short for "Piecewise Cubic Hermite
   * Interpolating Polynomial"), and provides a nice description in their
   * documentation.
   * http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf is also a good
   * reference.
   *
   * If `zero_end_point_derivatives` is `false`, the first and last first
   * derivative is chosen using a non-centered, shape-preserving three-point
   * formulae. See equation (2.10) in the following reference for more details.
   * http://www.mi.sanu.ac.rs/~gvm/radovi/mon.pdf
   * If `zero_end_point_derivatives` is `true`, they are set to zeros.
   *
   * If `zero_end_point_derivatives` is `false`, `breaks` and `samples` must
   * have at least 3 elements for the algorithm to determine the first
   * derivatives.
   *
   * If `zero_end_point_derivatives` is `true`, `breaks` and `samples` may have
   * 2 or more elements. For the 2 elements case, the result is equivalent to
   * computing a cubic polynomial whose values are given by `samples`, and
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
  static PiecewisePolynomial<T> CubicShapePreserving(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      bool zero_end_point_derivatives = false);

  DRAKE_DEPRECATED("2020-07-01",
                   "Pchip has been renamed to CubicShapePreserving.")
  static PiecewisePolynomial<T> Pchip(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      bool zero_end_point_derivatives = false) {
    return CubicShapePreserving(breaks, samples, zero_end_point_derivatives);
  }

  /**
   * Version of CubicShapePreserving(breaks, samples,
   * zero_end_point_derivatives) that uses vector samples and Eigen VectorXd
   * and MatrixX<T> inputs. Each column of `samples` represents a sample point.
   *
   * @pre `samples.cols() == breaks.size()`.
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> CubicShapePreserving(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      bool zero_end_point_derivatives = false);

  DRAKE_DEPRECATED("2020-07-01",
                   "Pchip has been renamed to CubicShapePreserving.")
  static PiecewisePolynomial<T> Pchip(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      bool zero_end_point_derivatives = false) {
    return CubicShapePreserving(breaks, samples, zero_end_point_derivatives);
  }

  /**
   * Constructs a third order %PiecewisePolynomial using matrix samples.
   * The %PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at `breaks`.
   * `sample_dot_at_start` and `sample_dot_at_end` are used for the first and
   * last first derivatives.
   *
   * @throws std::runtime_error if `sample_dot_at_start` or `sample_dot_at_end`
   * and `samples` have inconsistent dimensions.
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> CubicWithContinuousSecondDerivatives(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      const MatrixX<T>& sample_dot_at_start,
      const MatrixX<T>& sample_dot_at_end);

  DRAKE_DEPRECATED("2020-07-01", "This version of Cubic has been renamed to "
                                 "CubicWithContinuousSecondDerivatives.")
  static PiecewisePolynomial<T> Cubic(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      const MatrixX<T>& sample_dot_at_start,
      const MatrixX<T>& sample_dot_at_end) {
    return CubicWithContinuousSecondDerivatives(
        breaks, samples, sample_dot_at_start, sample_dot_at_end);
  }

  /**
   * Version of CubicWithContinuousSecondDerivatives() that uses vector
   * samples and Eigen VectorXd / MatrixX<T> inputs. Each column of
   * `samples` represents a sample point.
   *
   * @pre `samples.cols() == breaks.size()`.
   * @throws std::runtime_error under the conditions specified under
   *         @ref coefficient_construction_methods.
   */
  static PiecewisePolynomial<T> CubicWithContinuousSecondDerivatives(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      const Eigen::Ref<const VectorX<T>>& sample_dot_at_start,
      const Eigen::Ref<const VectorX<T>>& sample_dot_at_end);

  DRAKE_DEPRECATED("2020-07-01", "This version of Cubic has been renamed to "
  "CubicWithContinuousSecondDerivatives.")
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      const Eigen::Ref<const VectorX<T>>& sample_dot_at_start,
      const Eigen::Ref<const VectorX<T>>& sample_dot_at_end) {
    return CubicWithContinuousSecondDerivatives(
        breaks, samples, sample_dot_at_start, sample_dot_at_end);
  }

  /**
   * Constructs a third order %PiecewisePolynomial using matrix samples and
   * derivatives of samples (`samples_dot`); each matrix element of `samples_dot`
   * represents the derivative with respect to the independent variable (e.g.,
   * the time derivative) of the corresponding entry in `samples`.
   * Each segment is fully specified by `samples` and `sample_dot` at both ends.
   * Second derivatives are not continuous.
   *
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> CubicHermite(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      const std::vector<MatrixX<T>>& samples_dot);

  DRAKE_DEPRECATED("2020-07-01", "This version of Cubic has been renamed to "
  "CubicHermite.")
  static PiecewisePolynomial<T> Cubic(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      const std::vector<MatrixX<T>>& samples_dot) {
    return CubicHermite(breaks, samples, samples_dot);
  }

  /**
   * Version of CubicHermite(breaks, samples, samples_dot) that uses vector samples and
   * Eigen VectorXd / MatrixX<T> inputs. Corresponding columns of `samples` and
   * `samples_dot` are used as the sample point and independent variable derivative,
   * respectively.
   *
   * @pre `samples.cols() == samples_dot.cols() == breaks.size()`.
   */
  static PiecewisePolynomial<T> CubicHermite(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      const Eigen::Ref<const MatrixX<T>>& samples_dot);

  DRAKE_DEPRECATED("2020-07-01", "This version of Cubic has been renamed to "
  "CubicHermite.")
  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      const Eigen::Ref<const MatrixX<T>>& samples_dot) {
    return CubicHermite(breaks, samples, samples_dot);
  }

  /**
   * Constructs a third order %PiecewisePolynomial using matrix samples.
   * The %PiecewisePolynomial is constructed such that the interior segments
   * have the same value, first and second derivatives at `breaks`. If
   * `periodic_end_condition` is `false` (default), then the "Not-a-sample" end
   * condition is used here, which means the third derivatives are
   * continuous for the first two and last two segments. If
   * `periodic_end_condition` is `true`, then the first and second derivatives
   * between the end of the last segment and the beginning of the first
   * segment will be continuous. Note that the periodic end condition does
   * not require the first and last sample to be collocated, nor does it add
   * an additional sample to connect the first and last segments. Only first
   * and second derivative continuity is enforced.
   * See https://en.wikipedia.org/wiki/Spline_interpolation and
   * https://www.math.uh.edu/~jingqiu/math4364/spline.pdf
   * for more about cubic splines and their end conditions.
   * The MATLAB docs for methods "spline" and "csape" are also good
   * references.
   *
   * @pre `breaks` and `samples` must have at least 3 elements. If
   * `periodic_end_condition` is `true`, then for two samples, it would
   * produce a straight line (use `FirstOrderHold` for this instead), and if
   * `periodic_end_condition` is `false` the problem is ill-defined.
   * @exclude_from_pydrake_mkdoc{This overload is not bound in pydrake.}
   */
  static PiecewisePolynomial<T> CubicWithContinuousSecondDerivatives(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      bool periodic_end_condition = false);

  DRAKE_DEPRECATED("2020-07-01", "This version of Cubic has been renamed to "
  "CubicWithContinuousSecondDerivatives.")
  static PiecewisePolynomial<T> Cubic(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples,
      bool periodic_end_condition = false) {
    return CubicWithContinuousSecondDerivatives(breaks, samples,
                                                periodic_end_condition);
  }

  /**
   * Version of CubicWithContinuousSecondDerivatives(breaks, samples) that
   * uses vector samples and Eigen VectorXd / MatrixX<T> inputs. Each column
   * of `samples` represents a sample point.
   *
   * @pre `samples.cols() == breaks.size()`.
   */
  static PiecewisePolynomial<T> CubicWithContinuousSecondDerivatives(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      bool periodic_end_condition = false);

  static PiecewisePolynomial<T> Cubic(
      const Eigen::Ref<const VectorX<T>>& breaks,
      const Eigen::Ref<const MatrixX<T>>& samples,
      bool periodic_end_condition = false) {
    return CubicWithContinuousSecondDerivatives(breaks, samples,
        periodic_end_condition);
  }
  // @}

  /**
   * Returns a %PiecewisePolynomial where each segment is the specified
   * derivative of the corresponding segment in `this`. Any rules or limitations
   * of Polynomial::derivative() also apply to this function.
   *
   * Derivatives evaluated at non-differentiable points return the value at the
   * left hand side of the interval.
   * @param derivative_order The order of the derivative, namely, if
   *        `derivative_order` = n, the n'th derivative of the polynomial will
   *        be returned.
   * @warning In the event of discontinuous derivatives evaluated at breaks,
   *          it is not defined which polynomial (i.e., to the left or right
   *          of the break) will be the one that is evaluated at the break.
   */
  PiecewisePolynomial<T> derivative(int derivative_order = 1) const;

  std::unique_ptr<Trajectory<T>> MakeDerivative(
      int derivative_order = 1) const override {
    return derivative(derivative_order).Clone();
  };

  /**
   * Returns a %PiecewisePolynomial that is the indefinite integral of this one.
   * Any rules or limitations of Polynomial::integral() also apply to this
   * function.
   *
   * If `value_at_start_time` is given, it does the following only for the
   * first segment: adds that constant as the constant term
   * (zeroth-order coefficient) of the resulting Polynomial.
   */
  PiecewisePolynomial<T> integral(const T& value_at_start_time = 0.0) const;

  /**
   * Returns a %PiecewisePolynomial that is the indefinite integral of this one.
   * Any rules or limitations of Polynomial::integral() also apply to this
   * function.
   *
   * If `value_at_start_time` is given, it does the following only for the
   * first segment: adds `value_at_start_time(row,col)` as the constant term
   * (zeroth-order coefficient) of the resulting Polynomial.
   */
  PiecewisePolynomial<T> integral(
      const Eigen::Ref<MatrixX<T>>& value_at_start_time) const;

  /**
   * Returns `true` if this trajectory has no breaks/samples/polynomials.
   */
  bool empty() const { return polynomials_.empty(); }

  /**
   * Evaluates the trajectory at the given time without returning the entire
   * matrix. Equivalent to value(t)(row, col).
   * @warning See warnings in value().
   */
  T scalarValue(const T& t, Eigen::Index row = 0, Eigen::Index col = 0) const;

  /**
   * Evaluates the %PiecewisePolynomial at the given time t.
   *
   * @param t The time at which to evaluate the %PiecewisePolynomial.
   * @return The matrix of evaluated values.
   *
   * @warning If t does not lie in the range that the polynomial is defined
   *          over, the polynomial will silently be evaluated at the closest
   *          point to t. For example, `value(-1)` will return `value(0)` for a
   *          polynomial defined over [0, 1].
   * @warning This method only evaluates the polynomial in the segment defined
   *          by @p t.  If T=symbolic::Expression, then this method will return
   *          only the symbolic::Expression for the current segment if @p t can
   *          be cast to double or throw std::runtime_error if @p t cannot be
   *          cast to double.
   * @warning See warning in @ref polynomial_construction_warning.
   */
  MatrixX<T> value(const T& t) const override {
      const int derivative_order = 0;
      return EvalDerivative(t, derivative_order);
  }

  /**
   * Evaluates the %PiecwisePolynomial derivative at the given time @p t.
   * Returns the nth derivative, where `n` is the value of @p derivative_order.
   *
   * @warning This method comes with the same caveats as value(). See value().
   * @pre derivative_order must be non-negative.
   */
  MatrixX<T> EvalDerivative(const T& t, int derivative_order = 1) const;

  /**
   * Gets the matrix of Polynomials corresponding to the given segment index.
   * @warning `segment_index` is not checked for validity.
   */
  const PolynomialMatrix& getPolynomialMatrix(int segment_index) const;

  /**
   * Gets the Polynomial with the given matrix row and column index that
   * corresponds to the given segment index.
   * Equivalent to `getPolynomialMatrix(segment_index)(row, col)`.
   * @note Calls PiecewiseTrajectory<T>::segment_number_range_check() to
   *       validate `segment_index`.
   */
  const Polynomial<T>& getPolynomial(int segment_index, Eigen::Index row = 0,
                                      Eigen::Index col = 0) const;

  /**
   * Gets the degree of the Polynomial with the given matrix row and column
   * index that corresponds to the given segment index. Equivalent to
   * `getPolynomial(segment_index, row, col).GetDegree()`.
   */
  int getSegmentPolynomialDegree(int segment_index, Eigen::Index row = 0,
                                 Eigen::Index col = 0) const;

  /**
   * Returns the row count of the output matrices.
   * @throws std::runtime_error if empty().
   */
  Eigen::Index rows() const override;

  /**
   * Returns the column count of the output matrices.
   * @throws std::runtime_error if empty().
   */
  Eigen::Index cols() const override;

  /**
   * Adds each Polynomial in the PolynomialMatrix of `other` to the
   * corresponding Polynomial in the PolynomialMatrix of `this`, storing the
   * result in `this`. If `this` corresponds to t² and `other` corresponds to
   * t³, `this += other` will correspond to t³ + t².
   * @throws std::runtime_error if every element of `other.get_segment_times()`
   * is not within PiecewiseTrajectory::kEpsilonTime from
   * `this->get_segment_times().
   */
  PiecewisePolynomial& operator+=(const PiecewisePolynomial& other);

  /**
   * Subtracts each Polynomial in the PolynomialMatrix of `other` from the
   * corresponding Polynomial in the PolynomialMatrix of `this`, storing the
   * result in `this`. If `this` corresponds to t² and `other` corresponds to
   * t³, `this -= other` will correspond to t² - t³.
   * @throws std::runtime_error if every element of `other.get_segment_times()`
   * is not within PiecewiseTrajectory::kEpsilonTime from
   * `this->get_segment_times().
   */
  PiecewisePolynomial& operator-=(const PiecewisePolynomial& other);

  /**
   * Multiplies each Polynomial in the PolynomialMatrix of `other` by the
   * corresponding Polynomial in the PolynomialMatrix of `this` (i.e., a
   * coefficient-wise multiplication), storing the result in `this`. If `this`
   * corresponds to t² and `other` corresponds to t³, `this *= other` will
   * correspond to t⁵.
   * @throws std::runtime_error if every element of `other.get_segment_times()`
   * is not within PiecewiseTrajectory::kEpsilonTime from
   * `this->get_segment_times().
   */
  PiecewisePolynomial& operator*=(const PiecewisePolynomial& other);

  PiecewisePolynomial& operator+=(const MatrixX<T>& coeff);

  PiecewisePolynomial& operator-=(const MatrixX<T>& coeff);

  /**
   * Adds each Polynomial in the PolynomialMatrix of `other` to the
   * corresponding Polynomial in the PolynomialMatrix of `this`.
   * If `this` corresponds to t² and `other` corresponds to
   * t³, `this + other` will correspond to t³ + t².
   * @throws std::runtime_error if every element of `other.get_segment_times()`
   * is not within PiecewiseTrajectory::kEpsilonTime from
   * `this->get_segment_times().
   */
  const PiecewisePolynomial operator+(const PiecewisePolynomial& other) const;

  /**
   * Subtracts each Polynomial in the PolynomialMatrix of `other` from the
   * corresponding Polynomial in the PolynomialMatrix of `this`.
   * If `this` corresponds to t² and `other` corresponds to
   * t³, `this - other` will correspond to t² - t³.
   * @throws std::runtime_error if every element of `other.get_segment_times()`
   * is not within PiecewiseTrajectory::kEpsilonTime from
   * `this->get_segment_times().
   */
  const PiecewisePolynomial operator-(const PiecewisePolynomial& other) const;

  /**
   * Multiplies each Polynomial in the PolynomialMatrix of `other` by the
   * corresponding Polynomial in the PolynomialMatrix of `this` (i.e., a
   * coefficient-wise multiplication). If `this` corresponds to t² and `other`
   * corresponds to t³, `this *= other` will correspond to t⁵.
   * @throws std::runtime_error if every element of `other.get_segment_times()`
   * is not within PiecewiseTrajectory::kEpsilonTime from
   * `this->get_segment_times()1.
   */
  const PiecewisePolynomial operator*(const PiecewisePolynomial& other) const;

  const PiecewisePolynomial operator+(const MatrixX<T>& coeff) const;

  const PiecewisePolynomial operator-(const MatrixX<T>& coeff) const;

  // TODO(russt): Update return type to boolean<T> so that callers can obtain a
  // Formula when T=symbolic::Expression.
  /**
   * Checks whether a %PiecewisePolynomial is approximately equal to this one.
   *
   * Checks that every coefficient of `other` is within `tol` of the
   * corresponding coefficient of this %PiecewisePolynomial.
   * @throws std::exception if any Polynomial in either %PiecewisePolynomial is
   * not univariate.
   */
  bool isApprox(const PiecewisePolynomial& other, double tol) const;

  /**
   * Concatenates `other` to the end of `this`.
   *
   * @warning The resulting %PiecewisePolynomial will only be continuous to the
   *          degree that the first Polynomial of `other` is continuous with
   *          the last Polynomial of `this`. See warning about evaluating
   *          discontinuous derivatives at breaks in derivative().
   * @param other %PiecewisePolynomial instance to concatenate.
   * @throws std::runtime_error if trajectories' dimensions do not match
   *                            each other (either rows() or cols() does
   *                            not match between this and `other`).
   * @throws std::runtime_error if `this->end_time()` and `other->start_time()`
   *                            are not within
   *                            PiecewiseTrajectory<T>::kEpsilonTime from
   *                            each other.
   */
  void ConcatenateInTime(const PiecewisePolynomial& other);

  /**
   * The CubicHermite spline construction has a nice property of being
   * incremental (each segment can be solved independently). Given a new sample
   * and it's derivative, this method adds one segment to the end of `this`
   * where the start sample and derivative are taken as the value and derivative
   * at the final break of `this`.
   *
   * @pre `this` is not empty()
   * @pre `time` > end_time()
   * @pre `sample` and `sample_dot` must have size rows() x cols().
   */
  void AppendCubicHermiteSegment(
      const T& time, const Eigen::Ref<const MatrixX<T>>& sample,
      const Eigen::Ref<const MatrixX<T>>& sample_dot);

  /** Removes the final segment from the trajectory, reducing the number of
   * segments by 1.
   * @pre `this` is not empty()
   */
  void RemoveFinalSegment();

  /**
   * Modifies the trajectory so that pp_after(t) = pp_before(-t).
   *
   * @note The new trajectory will evaluate differently at precisely the break
   * points if the original trajectory was discontinuous at the break points.
   * This is because the segments are defined on the half-open intervals
   * [breaks(i), breaks(i+1)), and the order of the breaks have been reversed.
   */
  void ReverseTime();

  /**
   * Scales the time of the trajectory by non-negative `scale` (use
   * ReverseTime() if you want to also negate time). The resulting polynomial
   * evaluates to pp_after(t) = pp_before(t/scale).
   *
   * As an example, `scale`=2 will result in a trajectory that is twice as long
   * (start_time() and end_time() have both doubled).
   */
  void ScaleTime(const T& scale);

  /**
   * Adds `offset` to all of the breaks. `offset` need not be a non-negative
   * number.  The resulting polynomial will evaluate to pp_after(t) =
   * pp_before(t-offset).
   *
   * As an example, `offset`=2 will result in the start_time() and end_time()
   * being 2 seconds later.
   */
  void shiftRight(const T& offset);

  /**
   * Replaces the specified block of the PolynomialMatrix at the given
   * segment index.
   * @note Calls PiecewiseTrajectory<T>::segment_number_range_check() to
   *       validate `segment_index`.
   * @warning This code relies upon Eigen to verify that the replacement
   *          block is not too large.
   */
  void setPolynomialMatrixBlock(const PolynomialMatrix& replacement,
                                int segment_index, Eigen::Index row_start = 0,
                                Eigen::Index col_start = 0);

  /**
   * Returns the %PiecewisePolynomial comprising the `num_segments` segments
   * starting at the specified `start_segment_index`.
   * @note Calls PiecewiseTrajectory<T>::segment_number_range_check() to
   *       validate `segment_index`.
   */
  PiecewisePolynomial slice(int start_segment_index, int num_segments) const;

 private:
  T EvaluateSegmentAbsoluteTime(int segment_index, const T& t, Eigen::Index row,
                                Eigen::Index col,
                                int derivative_order = 0) const;

  // a PolynomialMatrix for each piece (segment).
  std::vector<PolynomialMatrix> polynomials_;

  // Computes coeffecients for a cubic spline given the value and first
  // derivatives at the end points.
  // Throws `std::runtime_error` if `dt < PiecewiseTrajectory::kEpsilonTime`.
  static Eigen::Matrix<T, 4, 1> ComputeCubicSplineCoeffs(const T& dt, T y0,
                                                         T y1, T yd0, T yd1);

  // For a cubic spline, there are 4 unknowns for each segment Pi, namely
  // the coefficients for Pi = a0 + a1 * t + a2 * t^2 + a3 * t^3.
  // Let N be the size of breaks and samples, there are N-1 segments,
  // and thus 4*(N-1) unknowns to fully specified a cubic spline for the given
  // data.
  //
  // If we are also given N sample_dot (velocity), each Pi will be fully
  // specified by (samples[i], sample_dot[i]) and (samples[i+1],
  // sample_dot[i+1]).  When sample_dot are not specified, we make the design
  // choice to enforce continuity up to the second order (Yddot) for
  // the interior points, i.e. Pi'(duration_i) = Pi+1'(0), and
  // Pi''(duration_i) = Pi+1''(0), where ' means time derivative, and
  // duration_i = breaks[i+1] - breaks[i] is the duration for the ith segment.
  //
  // At this point, we have 2 * (N - 1) position constraints:
  // Pi(0) = samples[i], for i in [0, N - 2]
  // Pi(duration_i) = samples[i+1], for i in [0, N - 2]
  // N - 2 velocity constraints for the interior points:
  // Pi'(duration_i) = Pi+1'(0), for i in [0, N - 3]
  // N - 2 acceleration constraints for the interior points:
  // Pi''(duration_i) = Pi+1''(0), for i in [0, N - 3]
  //
  // These sum up to 4 * (N - 1) - 2. This function sets up the above
  // constraints. There are still 2 constraints missing, which can be resolved
  // by various end point conditions (velocity at the end points /
  // "not-a-sample" / etc). These will be specified by the callers.
  static int SetupCubicSplineInteriorCoeffsLinearSystem(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples, int row, int col,
      MatrixX<T>* A, VectorX<T>* b);

  // Throws std::runtime_error if
  // `breaks` and `samples` have different length,
  // `breaks` is not strictly increasing,
  // `samples` has inconsistent dimensions,
  // `breaks` has length smaller than min_length.
  static void CheckSplineGenerationInputValidityOrThrow(
      const std::vector<T>& breaks,
      const std::vector<MatrixX<T>>& samples, int min_length);
};

}  // namespace trajectories
}  // namespace drake

DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class drake::trajectories::PiecewisePolynomial)
