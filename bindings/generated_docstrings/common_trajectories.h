#pragma once

// GENERATED FILE DO NOT EDIT
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#include <array>
#include <utility>

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

// #include "drake/common/trajectories/bezier_curve.h"
// #include "drake/common/trajectories/bspline_trajectory.h"
// #include "drake/common/trajectories/composite_trajectory.h"
// #include "drake/common/trajectories/derivative_trajectory.h"
// #include "drake/common/trajectories/discrete_time_trajectory.h"
// #include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
// #include "drake/common/trajectories/function_handle_trajectory.h"
// #include "drake/common/trajectories/path_parameterized_trajectory.h"
// #include "drake/common/trajectories/piecewise_constant_curvature_trajectory.h"
// #include "drake/common/trajectories/piecewise_polynomial.h"
// #include "drake/common/trajectories/piecewise_pose.h"
// #include "drake/common/trajectories/piecewise_quaternion.h"
// #include "drake/common/trajectories/piecewise_trajectory.h"
// #include "drake/common/trajectories/stacked_trajectory.h"
// #include "drake/common/trajectories/trajectory.h"
// #include "drake/common/trajectories/wrapped_trajectory.h"

// Symbol: pydrake_doc_common_trajectories
constexpr struct /* pydrake_doc_common_trajectories */ {
  // Symbol: drake
  struct /* drake */ {
    // Symbol: drake::trajectories
    struct /* trajectories */ {
      // Symbol: drake::trajectories::BezierCurve
      struct /* BezierCurve */ {
        // Source: drake/common/trajectories/bezier_curve.h
        const char* doc =
R"""(A Bézier curve is defined by a set of control points p₀ through pₙ,
where n is called the order of the curve (n = 1 for linear, 2 for
quadratic, 3 for cubic, etc.). The first and last control points are
always the endpoints of the curve; however, the intermediate control
points (if any) generally do not lie on the curve, but the curve is
guaranteed to stay within the convex hull of the control points.

See also BsplineTrajectory. A B-spline can be thought of as a
composition of overlapping Bézier curves (where each evaluation only
depends on a local subset of the control points). In contrast,
evaluating a Bézier curve will use all of the control points.)""";
        // Symbol: drake::trajectories::BezierCurve::AsLinearInControlPoints
        struct /* AsLinearInControlPoints */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Supports writing optimizations using the control points as decision
variables. This method returns the matrix, ``M``, defining the control
points of the ``order`` derivative in the form:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    derivative.control_points() = this.control_points() * M

.. raw:: html

    </details>

For instance, since we have


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    derivative.control_points().col(k) = this.control_points() * M.col(k),

.. raw:: html

    </details>

constraining the kth control point of the `n`th derivative to be in
[ub, lb], could be done with:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    auto M = curve.AsLinearInControlPoints(n);
    for (int i=0; i<curve.rows(); ++i) {
    prog.AddLinearConstraint(M.col(i).transpose(),
    Vector1d(lb(i)),
    Vector1d(ub(i)),
    curve.row(i).transpose());
    }

.. raw:: html

    </details>

Iterating over the rows of the control points is the natural sparsity
pattern here (since ``M`` is the same for all rows). For instance, we
also have


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    derivative.control_points().row(k).T = M.T * this.control_points().row(k).T,

.. raw:: html

    </details>

or


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    vec(derivative.control_points().T) = blockMT * vec(this.control_points().T),
    blockMT = [ M.T,   0, .... 0 ]
    [   0, M.T, 0, ... ]
    [      ...         ]
    [  ...    , 0, M.T ].

.. raw:: html

    </details>

Precondition:
    derivative_order >= 0.)""";
        } AsLinearInControlPoints;
        // Symbol: drake::trajectories::BezierCurve::BernsteinBasis
        struct /* BernsteinBasis */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Returns the value of the ith basis function of ``order`` (1 for
linear, 2 for quadratic, etc) evaluated at ``time``. The default value
for the optional argument ``order`` is the ``order()`` of ``this``.)""";
        } BernsteinBasis;
        // Symbol: drake::trajectories::BezierCurve::BezierCurve<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc_0args =
R"""(Default initializer. Constructs an empty Bézier curve over the
interval t ∈ [0, 1].)""";
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc_3args =
R"""(Constructs a Bézier curve over the interval t ∈ [`start_time`,
``end_time`] with control points defined in the columns of
`control_points``.

Precondition:
    end_time >= start_time.)""";
        } ctor;
        // Symbol: drake::trajectories::BezierCurve::ElevateOrder
        struct /* ElevateOrder */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Increases the order of the curve by 1. A Bézier curve of order n can
be converted into a Bézier curve of order n + 1 with the same shape.
The control points of ``this`` are modified to obtain the equivalent
curve.)""";
        } ElevateOrder;
        // Symbol: drake::trajectories::BezierCurve::GetExpression
        struct /* GetExpression */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Extracts the expanded underlying polynomial expression of this curve
in terms of variable ``time``.)""";
        } GetExpression;
        // Symbol: drake::trajectories::BezierCurve::control_points
        struct /* control_points */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Returns a const reference to the control points which define the
curve.)""";
        } control_points;
        // Symbol: drake::trajectories::BezierCurve::order
        struct /* order */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Returns the order of the curve (1 for linear, 2 for quadratic, etc.).)""";
        } order;
        // Symbol: drake::trajectories::BezierCurve::value
        struct /* value */ {
          // Source: drake/common/trajectories/bezier_curve.h
          const char* doc =
R"""(Evaluates the curve at the given time.

Warning:
    If t does not lie in the range [start_time(), end_time()], the
    trajectory will silently be evaluated at the closest valid value
    of time to ``time``. For example, ``value(-1)`` will return
    ``value(0)`` for a trajectory defined over [0, 1].)""";
        } value;
      } BezierCurve;
      // Symbol: drake::trajectories::BsplineTrajectory
      struct /* BsplineTrajectory */ {
        // Source: drake/common/trajectories/bspline_trajectory.h
        const char* doc =
R"""(Represents a B-spline curve using a given ``basis`` with ordered
``control_points`` such that each control point is a matrix in ℝʳᵒʷˢ ˣ
ᶜᵒˡˢ.

See also:
    math∷BsplineBasis)""";
        // Symbol: drake::trajectories::BsplineTrajectory::AsLinearInControlPoints
        struct /* AsLinearInControlPoints */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Supports writing optimizations using the control points as decision
variables. This method returns the matrix, ``M``, defining the control
points of the ``order`` derivative in the form:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    derivative.control_points() = this.control_points() * M

.. raw:: html

    </details>

See ``BezierCurve∷AsLinearInControlPoints()`` for more details.

Precondition:
    derivative_order >= 0.)""";
        } AsLinearInControlPoints;
        // Symbol: drake::trajectories::BsplineTrajectory::BsplineTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Constructs a B-spline trajectory with the given ``basis`` and
``control_points``.

Precondition:
    control_points.size() == basis.num_basis_functions())""";
        } ctor;
        // Symbol: drake::trajectories::BsplineTrajectory::CopyBlock
        struct /* CopyBlock */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Returns a new BsplineTrajectory that uses the same basis as ``this``,
and whose control points are the result of calling
``point.block(start_row, start_col, block_rows, block_cols)`` on each
``point`` in ``this->control_points()``.)""";
        } CopyBlock;
        // Symbol: drake::trajectories::BsplineTrajectory::CopyHead
        struct /* CopyHead */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Returns a new BsplineTrajectory that uses the same basis as ``this``,
and whose control points are the result of calling ``point.head(n)``
on each ``point`` in ``this->control_points()``.

Precondition:
    this->cols() == 1

Precondition:
    control_points()[0].head(n) must be a valid operation.)""";
        } CopyHead;
        // Symbol: drake::trajectories::BsplineTrajectory::CopyWithSelector
        struct /* CopyWithSelector */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Returns a new BsplineTrajectory that uses the same basis as ``this``,
and whose control points are the result of calling ``select(point)``
on each ``point`` in ``this->control_points()``.)""";
        } CopyWithSelector;
        // Symbol: drake::trajectories::BsplineTrajectory::EvaluateLinearInControlPoints
        struct /* EvaluateLinearInControlPoints */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Returns the vector, M, such that


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    EvalDerivative(t, derivative_order) = control_points() * M

.. raw:: html

    </details>

where cols()==1 (so control_points() is a matrix). This is useful for
writing linear constraints on the control points. Note that if the
derivative order is greater than or equal to the order of the basis,
then the result is a zero vector.

Precondition:
    t ≥ start_time()

Precondition:
    t ≤ end_time())""";
        } EvaluateLinearInControlPoints;
        // Symbol: drake::trajectories::BsplineTrajectory::FinalValue
        struct /* FinalValue */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc = R"""(Returns this->value(this->end_time()))""";
        } FinalValue;
        // Symbol: drake::trajectories::BsplineTrajectory::InitialValue
        struct /* InitialValue */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc = R"""(Returns this->value(this->start_time()))""";
        } InitialValue;
        // Symbol: drake::trajectories::BsplineTrajectory::InsertKnots
        struct /* InsertKnots */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Adds new knots at the specified ``additional_knots`` without changing
the behavior of the trajectory. The basis and control points of the
trajectory are adjusted such that it produces the same value for any
valid time before and after this method is called. The resulting
trajectory is guaranteed to have the same level of continuity as the
original, even if knot values are duplicated. Note that
``additional_knots`` need not be sorted.

Precondition:
    start_time() <= t <= end_time() for all t in ``additional_knots``)""";
        } InsertKnots;
        // Symbol: drake::trajectories::BsplineTrajectory::Serialize
        struct /* Serialize */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. This method is only available when T =
double.)""";
        } Serialize;
        // Symbol: drake::trajectories::BsplineTrajectory::basis
        struct /* basis */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc = R"""(Returns the basis of this curve.)""";
        } basis;
        // Symbol: drake::trajectories::BsplineTrajectory::control_points
        struct /* control_points */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Returns the control points of this curve.)""";
        } control_points;
        // Symbol: drake::trajectories::BsplineTrajectory::num_control_points
        struct /* num_control_points */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Returns the number of control points in this curve.)""";
        } num_control_points;
        // Symbol: drake::trajectories::BsplineTrajectory::value
        struct /* value */ {
          // Source: drake/common/trajectories/bspline_trajectory.h
          const char* doc =
R"""(Evaluates the BsplineTrajectory at the given time t.

Parameter ``t``:
    The time at which to evaluate the BsplineTrajectory.

Returns:
    The matrix of evaluated values.

Precondition:
    If T == symbolic∷Expression, ``t.is_constant()`` must be true.

Warning:
    If t does not lie in the range [start_time(), end_time()], the
    trajectory will silently be evaluated at the closest valid value
    of time to t. For example, ``value(-1)`` will return ``value(0)``
    for a trajectory defined over [0, 1].)""";
        } value;
      } BsplineTrajectory;
      // Symbol: drake::trajectories::CompositeTrajectory
      struct /* CompositeTrajectory */ {
        // Source: drake/common/trajectories/composite_trajectory.h
        const char* doc =
R"""(A "composite trajectory" is a series of trajectories joined end to end
where the end time of one trajectory coincides with the starting time
of the next.

See also PiecewisePolynomial∷ConcatenateInTime(), which might be
preferred if all of the segments are PiecewisePolynomial.)""";
        // Symbol: drake::trajectories::CompositeTrajectory::AlignAndConcatenate
        struct /* AlignAndConcatenate */ {
          // Source: drake/common/trajectories/composite_trajectory.h
          const char* doc =
R"""(Constructs a composite trajectory from a list of trajectories whose
start and end times may not coincide, by translating their start and
end times.

Precondition:
    ∀i, ``segments[i].get() != nullptr``.

Precondition:
    ∀i, ``segments[i].rows() == segments[0].rows()`` and
    ``segments[i].cols() == segments[0].cols()``.)""";
        } AlignAndConcatenate;
        // Symbol: drake::trajectories::CompositeTrajectory::CompositeTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/composite_trajectory.h
          const char* doc =
R"""(Constructs a composite trajectory from a list of Trajectories.

Precondition:
    ∀i, ``segments[i].get() != nullptr``.

Precondition:
    ∀i, ``segments[i+1].start_time() == segments[i].end_time()``.

Precondition:
    ∀i, ``segments[i].rows() == segments[0].rows()`` and
    ``segments[i].cols() == segments[0].cols()``.)""";
        } ctor;
        // Symbol: drake::trajectories::CompositeTrajectory::segment
        struct /* segment */ {
          // Source: drake/common/trajectories/composite_trajectory.h
          const char* doc =
R"""(Returns a reference to the ``segment_index`` trajectory.)""";
        } segment;
        // Symbol: drake::trajectories::CompositeTrajectory::value
        struct /* value */ {
          // Source: drake/common/trajectories/composite_trajectory.h
          const char* doc =
R"""(Evaluates the curve at the given time.

Warning:
    If t does not lie in the range [start_time(), end_time()], the
    trajectory will silently be evaluated at the closest valid value
    of time to ``time``. For example, ``value(-1)`` will return
    ``value(0)`` for a trajectory defined over [0, 1].)""";
        } value;
      } CompositeTrajectory;
      // Symbol: drake::trajectories::DerivativeTrajectory
      struct /* DerivativeTrajectory */ {
        // Source: drake/common/trajectories/derivative_trajectory.h
        const char* doc =
R"""(Trajectory objects provide derivatives by implementing
``DoEvalDerivative`` *and* ``DoMakeDerivative``. `DoEvalDerivative`
evaluates the derivative value at a point in time.
``DoMakeDerivative`` returns a new Trajectory object which represents
the derivative.

In some cases, it is easy to implement ``DoEvalDerivative``, but
difficult or inefficient to implement ``DoMakeDerivative`` natively.
And it may be just as efficient to use ``DoEvalDerivative`` even in
repeated evaluations of the derivative. The DerivativeTrajectory class
helps with this case -- given a ``nominal`` Trajectory, it provides a
Trajectory interface that calls ``nominal.EvalDerivative()`` to
implement ``Trajectory∷value()``.)""";
        // Symbol: drake::trajectories::DerivativeTrajectory::DerivativeTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/derivative_trajectory.h
          const char* doc =
R"""(Creates a DerivativeTrajectory representing the ``derivative_order``
derivatives of ``nominal``. This constructor makes a Clone() of
``nominal`` and does not hold on to the reference.

Raises:
    RuntimeError if ``!nominal.has_derivative()``.

Raises:
    RuntimeError if derivative_order < 0.)""";
        } ctor;
      } DerivativeTrajectory;
      // Symbol: drake::trajectories::DiscreteTimeTrajectory
      struct /* DiscreteTimeTrajectory */ {
        // Source: drake/common/trajectories/discrete_time_trajectory.h
        const char* doc =
R"""(A DiscreteTimeTrajectory is a Trajectory whose value is only defined
at discrete time points. Calling ``value()`` at a time that is not
equal to one of those times (up to a tolerance) will throw. This
trajectory does *not* have well-defined time-derivatives.

In some applications, it may be preferable to use
PiecewisePolynomial<T>∷ZeroOrderHold instead of a
DiscreteTimeTrajectory (and we offer a method here to easily convert).
Note if the breaks are periodic, then one can also achieve a similar
result in a Diagram by using the DiscreteTimeTrajectory in a
TrajectorySource and connecting a ZeroOrderHold system to the output
port, but remember that this will add discrete state to your diagram.

So why not always use the zero-order hold (ZOH) trajectory? This class
forces us to be more precise in our implementations. For instance,
consider the case of a solution to a discrete-time finite-horizon
linear quadratic regulator (LQR) problem. In this case, the solution
to the Riccati equation is a DiscreteTimeTrajectory, K(t).
Implementing


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t) -> MatrixGain(-K(t)) -> u(t)

.. raw:: html

    </details>

in a block diagram is perfectly correct, and if the u(t) is only
connected to the original system that it was designed for, then K(t)
will only get evaluated at the defined sample times, and all is well.
But if you wire it up to a continuous-time system, then K(t) may be
evaluated at arbitrary times, and may throw. If one wishes to use the
K(t) solution on a continuous-time system, then we can use


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    x(t) -> MatrixGain(-K(t)) -> ZOH -> u(t).

.. raw:: html

    </details>

This is different, and *more correct* than implementing K(t) as a
zero-order hold trajectory, because in this version, both K(t) and the
inputs x(t) will only be evaluated at the discrete-time input. If
``t_s`` was the most recent discrete sample time, then this means u(t)
= -K(t_s)*x(t_s) instead of u(t) = -K(t_s)*x(t). Using x(t_s) and
having a true zero-order hold on u(t) is the correct model for the
discrete-time LQR result.)""";
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::DiscreteTimeTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Default constructor creates the empty trajectory.)""";
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc_Eigen =
R"""(Constructs a trajectory of vector ``values`` at the specified
``times``.

Precondition:
    ``times`` must differ by more than ``time_comparison_tolerance``
    and be monotonically increasing.

Precondition:
    ``values`` must have times.size() columns.

Precondition:
    ``time_comparison_tolerance`` must be >= 0.

Raises:
    if T=symbolic:Expression and ``times`` are not constants.)""";
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc_stdvector =
R"""(Constructs a trajectory of matrix ``values`` at the specified
``times``.

Precondition:
    ``times`` should differ by more than ``time_comparison_tolerance``
    and be monotonically increasing.

Precondition:
    ``values`` must have times.size() elements, each with the same
    number of rows and columns.

Precondition:
    ``time_comparison_tolerance`` must be >= 0.

Raises:
    if T=symbolic:Expression and ``times`` are not constants.)""";
        } ctor;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::ToZeroOrderHold
        struct /* ToZeroOrderHold */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Converts the discrete-time trajectory using
PiecewisePolynomial<T>∷ZeroOrderHold().)""";
        } ToZeroOrderHold;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::cols
        struct /* cols */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the number of cols in the MatrixX<T> returned by value().

Precondition:
    num_times() > 0.)""";
        } cols;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::end_time
        struct /* end_time */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the maximum value of get_times().

Precondition:
    num_times() > 0.)""";
        } end_time;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::get_times
        struct /* get_times */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the times where the trajectory value is defined.)""";
        } get_times;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::num_times
        struct /* num_times */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the number of discrete times where the trajectory value is
defined.)""";
        } num_times;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::rows
        struct /* rows */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the number of rows in the MatrixX<T> returned by value().

Precondition:
    num_times() > 0.)""";
        } rows;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::start_time
        struct /* start_time */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the minimum value of get_times().

Precondition:
    num_times() > 0.)""";
        } start_time;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::time_comparison_tolerance
        struct /* time_comparison_tolerance */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(The trajectory is only defined at finite sample times. This method
returns the tolerance used determine which time sample (if any)
matches a query time on calls to value(t).)""";
        } time_comparison_tolerance;
        // Symbol: drake::trajectories::DiscreteTimeTrajectory::value
        struct /* value */ {
          // Source: drake/common/trajectories/discrete_time_trajectory.h
          const char* doc =
R"""(Returns the value of the trajectory at ``t``.

Raises:
    RuntimeError if t is not within tolerance of one of the sample
    times.)""";
        } value;
      } DiscreteTimeTrajectory;
      // Symbol: drake::trajectories::ExponentialPlusPiecewisePolynomial
      struct /* ExponentialPlusPiecewisePolynomial */ {
        // Source: drake/common/trajectories/exponential_plus_piecewise_polynomial.h
        const char* doc =
R"""(Represents a piecewise-trajectory with piece :math:`j` given by:

.. math:: x(t) = K e^{A (t - t_j)} \alpha_j + \sum_{i=0}^k \beta_{j,i}(t-t_j)^i,

where :math:`k` is the order of the ``piecewise_polynomial_part`` and
:math:`t_j` is the start time of the :math:`j`-th segment.

This particular form can represent the solution to a linear dynamical
system driven by a piecewise-polynomial input:

.. math:: \dot{x}(t) = A x(t) + B u(t),

where the input :math:`u(t)` is a piecewise-polynomial function of
time. See [1] for details and a motivating use case.

[1] R. Tedrake, S. Kuindersma, R. Deits and K. Miura, "A closed-form
solution for real-time ZMP gait generation and feedback
stabilization," 2015 IEEE-RAS 15th International Conference on
Humanoid Robots (Humanoids), Seoul, 2015, pp. 936-940.)""";
        // Symbol: drake::trajectories::ExponentialPlusPiecewisePolynomial::ExponentialPlusPiecewisePolynomial<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/exponential_plus_piecewise_polynomial.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::trajectories::ExponentialPlusPiecewisePolynomial::derivative
        struct /* derivative */ {
          // Source: drake/common/trajectories/exponential_plus_piecewise_polynomial.h
          const char* doc = R"""()""";
        } derivative;
        // Symbol: drake::trajectories::ExponentialPlusPiecewisePolynomial::shiftRight
        struct /* shiftRight */ {
          // Source: drake/common/trajectories/exponential_plus_piecewise_polynomial.h
          const char* doc = R"""()""";
        } shiftRight;
      } ExponentialPlusPiecewisePolynomial;
      // Symbol: drake::trajectories::FunctionHandleTrajectory
      struct /* FunctionHandleTrajectory */ {
        // Source: drake/common/trajectories/function_handle_trajectory.h
        const char* doc =
R"""(FunctionHandleTrajectory takes a function, value = f(t), and provides
a Trajectory interface.)""";
        // Symbol: drake::trajectories::FunctionHandleTrajectory::FunctionHandleTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/function_handle_trajectory.h
          const char* doc =
R"""(Creates the FunctionHandleTrajectory.

By default the created trajectory does not provide derivatives. If
trajectory derivatives are required, call ``set_derivative`` to
provide the function's derivatives.

Parameter ``func``:
    The function to be used to evaluate the trajectory.

Parameter ``rows``:
    The number of rows in the output of the function.

Parameter ``cols``:
    The number of columns in the output of the function.

Parameter ``start_time``:
    The start time of the trajectory.

Parameter ``end_time``:
    The end time of the trajectory.

Raises:
    RuntimeError if func == nullptr, rows < 0, cols < 0, start_time >
    end_time, or if the function returns a matrix of the wrong size.)""";
        } ctor;
        // Symbol: drake::trajectories::FunctionHandleTrajectory::set_derivative
        struct /* set_derivative */ {
          // Source: drake/common/trajectories/function_handle_trajectory.h
          const char* doc =
R"""(Sets a callback function that returns the derivative of the function.
``func(t,order)`` will only be called with ``order > 0``. It is
recommended that if the derivatives are not implemented for the
requested order, the callback should throw an exception.

The size of the output of ``func`` will be checked each time the
derivative is evaluated, and a RuntimeError will be thrown if the size
is incorrect.)""";
        } set_derivative;
      } FunctionHandleTrajectory;
      // Symbol: drake::trajectories::PathParameterizedTrajectory
      struct /* PathParameterizedTrajectory */ {
        // Source: drake/common/trajectories/path_parameterized_trajectory.h
        const char* doc =
R"""(A trajectory defined by a path and timing trajectory.

Using a path of form ``r(s)`` and a time_scaling of the form ``s(t)``,
a full trajectory of form ``q(t) = r(s(t))`` is modeled.)""";
        // Symbol: drake::trajectories::PathParameterizedTrajectory::PathParameterizedTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/path_parameterized_trajectory.h
          const char* doc =
R"""(Constructs a trajectory with the given ``path`` and ``time_scaling``.

Precondition:
    time_scaling.rows() == time_scaling.cols() == 1)""";
        } ctor;
        // Symbol: drake::trajectories::PathParameterizedTrajectory::path
        struct /* path */ {
          // Source: drake/common/trajectories/path_parameterized_trajectory.h
          const char* doc = R"""(Returns the path of this trajectory.)""";
        } path;
        // Symbol: drake::trajectories::PathParameterizedTrajectory::time_scaling
        struct /* time_scaling */ {
          // Source: drake/common/trajectories/path_parameterized_trajectory.h
          const char* doc =
R"""(Returns the time_scaling of this trajectory.)""";
        } time_scaling;
        // Symbol: drake::trajectories::PathParameterizedTrajectory::value
        struct /* value */ {
          // Source: drake/common/trajectories/path_parameterized_trajectory.h
          const char* doc =
R"""(Evaluates the PathParameterizedTrajectory at the given time t.

Parameter ``t``:
    The time at which to evaluate the PathParameterizedTrajectory.

Returns:
    The matrix of evaluated values.

Precondition:
    If T == symbolic∷Expression, ``t.is_constant()`` must be true.

Warning:
    If t does not lie in the range [start_time(), end_time()], the
    trajectory will silently be evaluated at the closest valid value
    of time to t. For example, ``value(-1)`` will return ``value(0)``
    for a trajectory defined over [0, 1].)""";
        } value;
      } PathParameterizedTrajectory;
      // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory
      struct /* PiecewiseConstantCurvatureTrajectory */ {
        // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
        const char* doc =
R"""(A piecewise constant curvature trajectory in a plane, where the plane
is posed arbitrarily in three dimensions.

The trajectory is defined by the position vector r(s): ℝ → ℝ³,
parameterized by arclength s, and lies in a plane with normal vector
p̂. The parameterization is C¹, i.e. position r(s) and tangent vector
t̂(s) = dr(s)/ds are continuous functions of arclength s. The
trajectory's length is divided into segments s₀ < s₁ < ... < sₙ, where
each interval [sᵢ, sᵢ₊₁) has constant curvature determined by the
turning rate ρᵢ (with units of 1/m). The turning rate is defined such
that curvature is κᵢ = |ρᵢ|, with its sign indicating the curve's
direction around p̂ according ot the right-hand rule (counterclockwise
if positive and clockwise if negative) . For ρᵢ = 0, the segment is a
straight line.

Given the tangent vector t̂(s) = dr(s)/ds and the plane's normal p̂,
we define the *normal* vector along the curve as n̂(s) = p̂ × t̂(s).
These three vectors are used to define a moving frame M along the
curve with basis vectors Mx, My, Mz coincident with vectors t̂, n̂,
p̂, respectively.

A trajectory is said to be periodic if the pose X_AM of frame M is a
periodic function of arclength, i.e. X_AM(s) = X_AM(s + k⋅L) ∀ k ∈ ℤ,
where L is the length of a single cycle along the trajectory.
Periodicity is determined at construction.

Note:
    Though similar, frame M is distinct from the Frenet–Serret frame
    defined by the tangent-normal-binormal vectors T̂, N̂, B̂ See <a
    href="https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas">Frenet–Serret
    formulas</a> for further reading.

For constant curvature paths on a plane, the <a
href="https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas">Frenet–Serret
formulas</a> simplify and we can write:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    dMx/ds(s) =  ρ(s)⋅My(s)
    dMy/ds(s) = -ρ(s)⋅Mx(s)
    dMz/ds(s) =  0

.. raw:: html

    </details>

for the entire trajectory.

The spatial orientation of the curve is defined by the plane's normal
p̂ and the initial tangent vector t̂₀ at s = 0. At construction, these
are provided in a reference frame A, defining the pose X_AM₀ at s = 0.
This class provides functions to compute the curve's kinematics given
by its pose X_AM (comprising position vector r(s) = p_AoMo_A(s) and
orientation R_AM(s)), spatial velocity V_AM_A(s) and spatial
acceleration A_AM_A(s). We also provide functions to return these
quantities in the M frame (where they are much simpler), i.e.
V_AM_M(s) and A_AM_M(s).

Warning:
    Note that this class models a curve parameterized by arclength,
    rather than time as the Trajectory class and many of its
    inheriting types assume. Time derivatives, i.e. spatial velocity
    and acceleration, are computed for externally provided values of
    velocity ṡ and acceleration s̈ along the curve.

See also:
    multibody∷CurvilinearJoint)""";
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::CalcPose
        struct /* CalcPose */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Calculates the trajectory's pose X_AM(s) at the given arclength s.

Note:
    If the trajectory is aperiodic, for s < 0 and s > length() the
    pose is extrapolated as if the curve continued with the curvature
    of the corresponding end segment.

Parameter ``s``:
    The query arclength in meters.

Returns:
    the pose X_AM(s).)""";
        } CalcPose;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::CalcSpatialAcceleration
        struct /* CalcSpatialAcceleration */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Computes the spatial acceleration A_AM_A(s,ṡ,s̈) of frame M measured
and expressed in the reference frame A. See the class's documentation
for frame definitions.

In frame invariant notation, the angular acceleration α(s) and
translational acceleration a(s) are:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    α(s) = s̈⋅ρ(s)⋅p̂
    a(s) = ṡ²⋅ρ(s)⋅n̂(s) + s̈⋅t̂(s)

.. raw:: html

    </details>

where ρ(s), t̂(s) and n̂(s) are extrapolated for s < 0 and s >
length() keeping the constant curvature of the corresponding end
segment.

As the curve does not have continuous acceleration at the breaks, by
convention we set the acceleration at the break sᵢ to be the limit as
approached from the right -- i.e. the acceleration is continuous on
each segment domain [sᵢ, sᵢ₊₁).

Parameter ``s``:
    The query arclength, in meters.

Parameter ``s_dot``:
    The magnitude ṡ of the tangential velocity along the curve, in
    meters per second.

Parameter ``s_ddot``:
    The magnitude s̈ of the tangential acceleration along the curve,
    in meters per second squared.

Returns ``A_AM_A``:
    The spatial acceleration of M in A, expressed in A.)""";
        } CalcSpatialAcceleration;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::CalcSpatialAccelerationInM
        struct /* CalcSpatialAccelerationInM */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Computes the spatial acceleration A_AM_M(s,ṡ,s̈) of frame M measured
in frame A but expressed in frame M.

See CalcSpatialAcceleration() for details. Note that when expressed in
frame M, the vectors p̂, t̂, and n̂ are constant, with p̂_M = [0 0
1]ᵀ, t̂_M = [1 0 0]ᵀ, and n̂_M = [0 1 0]ᵀ so the returned spatial
acceleration is just ``A_AM_M = [0 0 s̈⋅ρ(s) s̈ ṡ²⋅ρ(s) 0]ᵀ``.

Parameter ``s``:
    The query arclength, in meters.

Parameter ``s_dot``:
    The magnitude ṡ of the tangential velocity along the curve, in
    meters per second.

Parameter ``s_ddot``:
    The magnitude s̈ of the tangential acceleration along the curve,
    in meters per second squared.

Returns ``A_AM_M``:
    The spatial acceleration of M in A, expressed in M.)""";
        } CalcSpatialAccelerationInM;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::CalcSpatialVelocity
        struct /* CalcSpatialVelocity */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Computes the spatial velocity V_AM_A(s,ṡ) of frame M measured and
expressed in the reference frame A. See the class's documentation for
frame definitions.

In frame invariant notation, the angular velocity ω(s) and
translational velocity v(s) are:


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    ω(s) = ṡ⋅ρ(s)⋅p̂
    v(s) = ṡ⋅t̂(s)

.. raw:: html

    </details>

where ρ(s) and t̂(s) are extrapolated for s < 0 and s > length()
keeping the constant curvature of the corresponding end segment.

Parameter ``s``:
    The query arclength, in meters.

Parameter ``s_dot``:
    The magnitude ṡ of the tangential velocity along the curve, in
    meters per second.

Returns ``V_AM_A``:
    The spatial velocity of M in A, expressed in A.)""";
        } CalcSpatialVelocity;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::CalcSpatialVelocityInM
        struct /* CalcSpatialVelocityInM */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Computes the spatial velocity V_AM_M(s,ṡ) of frame M measured in
frame A but expressed in frame M.

See CalcSpatialVelocity() for details. Note that when expressed in
frame M, both p̂ and t̂ are constant, with p̂_M = [0 0 1]ᵀ and t̂_M =
[1 0 0]ᵀ so the returned spatial velocity is just ``V_AM_M = [0 0
ṡ⋅ρ(s) ṡ 0 0]ᵀ``.

Parameter ``s``:
    The query arclength, in meters.

Parameter ``s_dot``:
    The magnitude ṡ of the tangential velocity along the curve, in
    meters per second.

Returns ``V_AM_M``:
    The spatial velocity of M in A, expressed in M.)""";
        } CalcSpatialVelocityInM;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::EndpointsAreNearlyEqual
        struct /* EndpointsAreNearlyEqual */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Returns:
    ``True`` if the trajectory is periodic within a given
    ``tolerance``.

Periodicity is defined as the beginning and end poses X_AM(s₀) and
X_AM(sₙ) being equal up to the same tolerance, checked via
RigidTransform∷IsNearlyEqualTo() using ``tolerance``.

Parameter ``tolerance``:
    The tolerance for the pose equality check.)""";
        } EndpointsAreNearlyEqual;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::PiecewiseConstantCurvatureTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc_0args =
R"""(An empty piecewise constant curvature trajectory.)""";
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc_6args =
R"""(Constructs a piecewise constant curvature trajectory.

Endpoints of each constant-curvature segments are defined by n breaks
s₀ = 0 < s₁ < ... < sₙ (in meters). The turning rates ρ₀, ..., ρₙ₋₁
(in 1/m) are passed through ``turning_rates``. There must be exactly
one turning rate per segment, i.e. turning_rates.size() ==
breaks.size() - 1.

Warning:
    Users of this class are responsible for providing
    ``initial_curve_tangent``, `plane_normal` and ``initial_position``
    expressed in the same reference frame A.

The ``initial_curve_tangent`` t̂₀ and the ``plane_normal`` p̂, along
with the initial curve's normal n̂₀ = p̂ × t̂₀, define the initial
orientation R_AM₀ of frame M at s = 0.

Parameter ``breaks``:
    A vector of n break values sᵢ between segments. The parent class,
    PiecewiseTrajectory, enforces that the breaks increase by at least
    PiecewiseTrajectory∷kEpsilonTime.

Parameter ``turning_rates``:
    A vector of n-1 turning rates ρᵢ for each segment.

Parameter ``initial_curve_tangent``:
    The initial tangent of the curve expressed in the parent frame,
    t̂_A(s₀) = Mx_A(s₀).

Parameter ``plane_normal``:
    The normal axis of the 2D plane in which the curve lies, expressed
    in the parent frame, p̂_A = Mz_A (constant).

Parameter ``initial_position``:
    The initial position of the curve expressed in the parent frame,
    p_AoMo_A(s₀).

Parameter ``is_periodic``:
    If true, then the newly constructed trajectory will be periodic.
    That is, X_AM(s) = X_AM(s + k⋅L) ∀ k ∈ ℤ, where L equals length().
    Subsequent calls to is_periodic() will return ``True``.

Raises:
    RuntimeError if the number of turning rates does not match the
    number of segments

Raises:
    RuntimeError if or s₀ is not 0.

Raises:
    RuntimeError if initial_curve_tangent or plane_normal have zero
    norm.

Raises:
    RuntimeError if initial_curve_tangent is not perpendicular to
    plane_normal.)""";
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc_1args =
R"""(Scalar conversion constructor. See system_scalar_conversion.)""";
        } ctor;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::curvature
        struct /* curvature */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Returns the signed curvature ρ(s).

Parameter ``s``:
    The query arclength in meters.

Returns:
    curvature ρ(s))""";
        } curvature;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::is_periodic
        struct /* is_periodic */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Returns:
    ``True`` if ``this`` trajectory is periodic. That is, X_AM(s) =
    X_AM(s + k⋅L) ∀ k ∈ ℤ, where L equals length().)""";
        } is_periodic;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::length
        struct /* length */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Returns:
    the total arclength of the curve in meters.)""";
        } length;
        // Symbol: drake::trajectories::PiecewiseConstantCurvatureTrajectory::value
        struct /* value */ {
          // Source: drake/common/trajectories/piecewise_constant_curvature_trajectory.h
          const char* doc =
R"""(Computes r(s), the trajectory's position p_AoMo_A(s) expressed in
reference frame A, at the given arclength s.

Parameter ``s``:
    The query arclength in meters.

Returns:
    position vector r(s).)""";
        } value;
      } PiecewiseConstantCurvatureTrajectory;
      // Symbol: drake::trajectories::PiecewisePolynomial
      struct /* PiecewisePolynomial */ {
        // Source: drake/common/trajectories/piecewise_polynomial.h
        const char* doc =
R"""(A scalar multi-variate piecewise polynomial.

PiecewisePolynomial represents a list of contiguous segments in a
scalar independent variable (typically corresponding to time) with
Polynomials defined at each segment. We call the output from
evaluating the PiecewisePolynomial at the scalar independent variable
"the output", and that output can be either a Eigen MatrixX<T> (if
evaluated using value()) or a scalar (if evaluated using
scalar_value()).

An example of a piecewise polynomial is a function of m segments in
time, where a different polynomial is defined for each segment. For a
specific example, consider the absolute value function over the
interval [-1, 1]. We can define a PiecewisePolynomial over this
interval using breaks at t = { -1.0, 0.0, 1.0 }, and "samples" of
abs(t).


.. raw:: html

    <details><summary>Click to expand C++ code...</summary>

.. code-block:: c++

    // Construct the PiecewisePolynomial.
    const std∷vector<double> breaks = { -1.0, 0.0, 1.0 };
    std∷vector<Eigen∷MatrixXd> samples(3);
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
      samples[i].resize(1, 1);
      samples[i](0, 0) = std∷abs(breaks[i]);
    }
    const auto pp =
         PiecewisePolynomial<double>∷FirstOrderHold(breaks, samples);
    const int row = 0, col = 0;
    
    // Evaluate the PiecewisePolynomial at some values.
    std∷cout << pp.value(-.5)(row, col) << std∷endl;    // Outputs 0.5.
    std∷cout << pp.value(0.0)(row, col) << std∷endl;    // Outputs 0.0;
    
    // Show how we can evaluate the first derivative (outputs -1.0).
    std∷cout << pp.derivative(1).value(-.5)(row, col) << std∷endl;

.. raw:: html

    </details>

A note on terminology. For piecewise-polynomial interpolation, we use
``breaks`` to indicate the scalar (e.g. times) which form the boundary
of each segment. We use ``samples`` to indicate the function value at
the ``breaks``, e.g. ``p(breaks[i]) = samples[i]``. The term ``knot``
should be reserved for the "(x,y)" coordinate, here ``knot[i] =
(breaks[i], samples[i])``, though it is used inconsistently in the
interpolation literature (sometimes for ``breaks``, sometimes for
``samples``), so we try to mostly avoid it here.

PiecewisePolynomial objects can be added, subtracted, and multiplied.
They cannot be divided because Polynomials are not closed under
division.

Warning:
    PiecewisePolynomial silently clips input evaluations outside of
    the range defined by the breaks. So ``pp.value(-2.0, row, col)``
    in the example above would evaluate to -1.0. See value().)""";
        // Symbol: drake::trajectories::PiecewisePolynomial::AddBreak
        struct /* AddBreak */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Adds a break at the specified time. It does not change the value of
the trajectory at any point but the number of segments increases by 1.

Returns:
    the index of the new break.

Raises:
    RuntimeError if ``new_break`` is not within the trajectory's time
    range.

Warning:
    If ``new_break`` is within PiecewiseTrajectory∷kEpsilonTime from
    an existing break, the new break will be silently ignored. Returns
    the index of the existing break.)""";
        } AddBreak;
        // Symbol: drake::trajectories::PiecewisePolynomial::AppendCubicHermiteSegment
        struct /* AppendCubicHermiteSegment */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(The CubicHermite spline construction has a nice property of being
incremental (each segment can be solved independently). Given a new
sample and it's derivative, this method adds one segment to the end of
``this`` where the start sample and derivative are taken as the value
and derivative at the final break of ``this``.

Precondition:
    ``this`` is not empty()

Precondition:
    ``time`` > end_time()

Precondition:
    ``sample`` and ``sample_dot`` must have size rows() x cols().)""";
        } AppendCubicHermiteSegment;
        // Symbol: drake::trajectories::PiecewisePolynomial::AppendFirstOrderSegment
        struct /* AppendFirstOrderSegment */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Given a new sample, this method adds one segment to the end of
``this`` using a first-order hold, where the start sample is taken as
the value at the final break of ``this``.)""";
        } AppendFirstOrderSegment;
        // Symbol: drake::trajectories::PiecewisePolynomial::Block
        struct /* Block */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Extracts a trajectory representing a block of size (block_rows,
block_cols) starting at (start_row, start_col) from the
PiecewisePolynomial.

Returns:
    a PiecewisePolynomial such that ret.value(t) =
    this.value(t).block(i,j,p,q);)""";
        } Block;
        // Symbol: drake::trajectories::PiecewisePolynomial::ConcatenateInTime
        struct /* ConcatenateInTime */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Concatenates ``other`` to the end of ``this``.

Warning:
    The resulting PiecewisePolynomial will only be continuous to the
    degree that the first Polynomial of ``other`` is continuous with
    the last Polynomial of ``this``. See warning about evaluating
    discontinuous derivatives at breaks in derivative().

Parameter ``other``:
    PiecewisePolynomial instance to concatenate.

Raises:
    RuntimeError if trajectories' dimensions do not match each other
    (either rows() or cols() does not match between this and
    ``other``).

Raises:
    RuntimeError if ``this->end_time()`` and ``other->start_time()``
    are not within PiecewiseTrajectory<T>∷kEpsilonTime from each
    other.)""";
        } ConcatenateInTime;
        // Symbol: drake::trajectories::PiecewisePolynomial::CubicHermite
        struct /* CubicHermite */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_matrix =
R"""(Constructs a third order PiecewisePolynomial using matrix samples and
derivatives of samples (``samples_dot``); each matrix element of
``samples_dot`` represents the derivative with respect to the
independent variable (e.g., the time derivative) of the corresponding
entry in ``samples``. Each segment is fully specified by ``samples``
and ``sample_dot`` at both ends. Second derivatives are not
continuous.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_vector =
R"""(Version of CubicHermite(breaks, samples, samples_dot) that uses vector
samples and Eigen VectorXd / MatrixX<T> arguments. Corresponding
columns of ``samples`` and ``samples_dot`` are used as the sample
point and independent variable derivative, respectively.

Precondition:
    ``samples.cols() == samples_dot.cols() == breaks.size()``.)""";
        } CubicHermite;
        // Symbol: drake::trajectories::PiecewisePolynomial::CubicShapePreserving
        struct /* CubicShapePreserving */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_matrix =
R"""(Constructs a third order PiecewisePolynomial using vector samples,
where each column of ``samples`` represents a sample point. First
derivatives are chosen to be "shape preserving", i.e. if ``samples``
is monotonic within some interval, the interpolated data will also be
monotonic. The second derivative is not guaranteed to be smooth across
the entire spline.

MATLAB calls this method "pchip" (short for "Piecewise Cubic Hermite
Interpolating Polynomial"), and provides a nice description in their
documentation.
http://home.uchicago.edu/~sctchoi/courses/cs138/interp.pdf is also a
good reference.

If ``zero_end_point_derivatives`` is ``False``, the first and last
first derivative is chosen using a non-centered, shape-preserving
three-point formulae. See equation (2.10) in the following reference
for more details. http://www.mi.sanu.ac.rs/~gvm/radovi/mon.pdf If
``zero_end_point_derivatives`` is ``True``, they are set to zeros.

If ``zero_end_point_derivatives`` is ``False``, `breaks` and
``samples`` must have at least 3 elements for the algorithm to
determine the first derivatives.

If ``zero_end_point_derivatives`` is ``True``, `breaks` and
``samples`` may have 2 or more elements. For the 2 elements case, the
result is equivalent to computing a cubic polynomial whose values are
given by ``samples``, and derivatives set to zero.

Raises:
    RuntimeError if: - ``breaks`` has length smaller than 3 and
    ``zero_end_point_derivatives`` is ``False``, - `breaks` has length
    smaller than 2 and ``zero_end_point_derivatives`` is true.

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_vector =
R"""(Version of CubicShapePreserving(breaks, samples,
zero_end_point_derivatives) that uses vector samples and Eigen
VectorXd and MatrixX<T> arguments. Each column of ``samples``
represents a sample point.

Precondition:
    ``samples.cols() == breaks.size()``.

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
        } CubicShapePreserving;
        // Symbol: drake::trajectories::PiecewisePolynomial::CubicWithContinuousSecondDerivatives
        struct /* CubicWithContinuousSecondDerivatives */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_4args_matrix =
R"""(Constructs a third order PiecewisePolynomial using matrix samples. The
PiecewisePolynomial is constructed such that the interior segments
have the same value, first and second derivatives at ``breaks``.
`sample_dot_at_start` and ``sample_dot_at_end`` are used for the first
and last first derivatives.

Raises:
    RuntimeError if ``sample_dot_at_start`` or ``sample_dot_at_end``
    and ``samples`` have inconsistent dimensions.

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_4args_vector =
R"""(Version of CubicWithContinuousSecondDerivatives() that uses vector
samples and Eigen VectorXd / MatrixX<T> arguments. Each column of
``samples`` represents a sample point.

Precondition:
    ``samples.cols() == breaks.size()``.

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_3args_matrix =
R"""(Constructs a third order PiecewisePolynomial using matrix samples. The
PiecewisePolynomial is constructed such that the interior segments
have the same value, first and second derivatives at ``breaks``. If
``periodic_end_condition`` is ``False`` (default), then the
"Not-a-sample" end condition is used here, which means the third
derivatives are continuous for the first two and last two segments. If
``periodic_end_condition`` is ``True``, then the first and second
derivatives between the end of the last segment and the beginning of
the first segment will be continuous. Note that the periodic end
condition does not require the first and last sample to be collocated,
nor does it add an additional sample to connect the first and last
segments. Only first and second derivative continuity is enforced. See
https://en.wikipedia.org/wiki/Spline_interpolation and
https://web.archive.org/web/20140125011904/https://www.math.uh.edu/~jingqiu/math4364/spline.pdf
for more about cubic splines and their end conditions. The MATLAB docs
for methods "spline" and "csape" are also good references.

Precondition:
    ``breaks`` and ``samples`` must have at least 3 elements. If
    ``periodic_end_condition`` is ``True``, then for two samples, it
    would produce a straight line (use ``FirstOrderHold`` for this
    instead), and if ``periodic_end_condition`` is ``False`` the
    problem is ill-defined.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_3args_vector =
R"""(Version of CubicWithContinuousSecondDerivatives(breaks, samples) that
uses vector samples and Eigen VectorXd / MatrixX<T> arguments. Each
column of ``samples`` represents a sample point.

Precondition:
    ``samples.cols() == breaks.size()``.)""";
        } CubicWithContinuousSecondDerivatives;
        // Symbol: drake::trajectories::PiecewisePolynomial::FirstOrderHold
        struct /* FirstOrderHold */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_matrix =
R"""(Constructs a piecewise linear PiecewisePolynomial using matrix
samples.

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_vector =
R"""(Version of FirstOrderHold(breaks, samples) that uses vector samples
and Eigen VectorXd / MatrixX<T> arguments. Each column of ``samples``
represents a sample point.

Precondition:
    ``samples.cols() == breaks.size()``

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
        } FirstOrderHold;
        // Symbol: drake::trajectories::PiecewisePolynomial::LagrangeInterpolatingPolynomial
        struct /* LagrangeInterpolatingPolynomial */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_matrix =
R"""(Constructs a polynomial with a *single segment* of the lowest possible
degree that passes through all of the sample points. See "polynomial
interpolation" and/or "Lagrange polynomial" on Wikipedia for more
information.

Precondition:
    ``times`` must be monotonically increasing.

Precondition:
    ``samples.size() == times.size()``.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_vector =
R"""(Version of LagrangeInterpolatingPolynomial(times, samples) that uses
vector samples and Eigen VectorXd / MatrixX<T> arguments. Each column
of ``samples`` represents a sample point.

Precondition:
    ``samples.cols() == times.size()``.)""";
        } LagrangeInterpolatingPolynomial;
        // Symbol: drake::trajectories::PiecewisePolynomial::PiecewisePolynomial<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_0args =
R"""(Constructs an empty piecewise polynomial.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_1args_constEigenMatrixBase =
R"""(Single segment, constant value constructor over the interval [-∞, ∞].
The constructed PiecewisePolynomial will return ``constant_value`` at
every evaluated point (i.e., ``value(t) = constant_value`` ∀t ∈ [-∞,
∞]).)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_2args_polynomials_matrix_breaks =
R"""(Constructs a PiecewisePolynomial using matrix-output Polynomials
defined over each segment.

Precondition:
    ``polynomials.size() == breaks.size() - 1``)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_2args_polynomials_breaks =
R"""(Constructs a PiecewisePolynomial using scalar-output Polynomials
defined over each segment.

Precondition:
    ``polynomials.size() == breaks.size() - 1``)""";
        } ctor;
        // Symbol: drake::trajectories::PiecewisePolynomial::PolynomialMatrix
        struct /* PolynomialMatrix */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc = R"""()""";
        } PolynomialMatrix;
        // Symbol: drake::trajectories::PiecewisePolynomial::RemoveFinalSegment
        struct /* RemoveFinalSegment */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Removes the final segment from the trajectory, reducing the number of
segments by 1.

Precondition:
    ``this`` is not empty())""";
        } RemoveFinalSegment;
        // Symbol: drake::trajectories::PiecewisePolynomial::Reshape
        struct /* Reshape */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Reshapes the dimensions of the Eigen∷MatrixX<T> returned by value(),
EvalDerivative(), etc.

Precondition:
    ``rows`` x ``cols`` must equal this.rows() * this.cols().

See also:
    Eigen∷PlainObjectBase∷resize().)""";
        } Reshape;
        // Symbol: drake::trajectories::PiecewisePolynomial::ReverseTime
        struct /* ReverseTime */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Modifies the trajectory so that pp_after(t) = pp_before(-t).

Note:
    The new trajectory will evaluate differently at precisely the
    break points if the original trajectory was discontinuous at the
    break points. This is because the segments are defined on the
    half-open intervals [breaks(i), breaks(i+1)), and the order of the
    breaks have been reversed.)""";
        } ReverseTime;
        // Symbol: drake::trajectories::PiecewisePolynomial::ScaleTime
        struct /* ScaleTime */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Scales the time of the trajectory by non-negative ``scale`` (use
ReverseTime() if you want to also negate time). The resulting
polynomial evaluates to pp_after(t) = pp_before(t/scale).

As an example, `scale`=2 will result in a trajectory that is twice as
long (start_time() and end_time() have both doubled).)""";
        } ScaleTime;
        // Symbol: drake::trajectories::PiecewisePolynomial::Serialize
        struct /* Serialize */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Passes this object to an Archive. Refer to yaml_serialization "YAML
Serialization" for background. This method is only available when T =
double.)""";
        } Serialize;
        // Symbol: drake::trajectories::PiecewisePolynomial::SliceByTime
        struct /* SliceByTime */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Slices the trajectory within a specified time range. q =
p.SliceByTime(t1, t2) returns a PiecewisePolynomial q such that
q.start_time() = t1, q.end_time() = t2, and q(t) = p(t) for t1 <= t <=
t2.

Raises:
    RuntimeError if ``start_time`` or ``end_time`` is not within the
    trajectory's time range.)""";
        } SliceByTime;
        // Symbol: drake::trajectories::PiecewisePolynomial::Transpose
        struct /* Transpose */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Constructs a new PiecewisePolynomial for which value(t) ==
this.value(t).transpose().)""";
        } Transpose;
        // Symbol: drake::trajectories::PiecewisePolynomial::ZeroOrderHold
        struct /* ZeroOrderHold */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_matrix =
R"""(Constructs a piecewise constant PiecewisePolynomial using matrix
samples. Note that constructing a PiecewisePolynomial requires at
least two sample points, although in this case, the second sample
point's value is ignored, and only its break time is used.

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_vector =
R"""(Version of ZeroOrderHold(breaks, samples) that uses vector samples and
``Eigen∷VectorX<T>/MatrixX<T>`` arguments. Each column of ``samples``
represents a sample point.

Precondition:
    ``samples.cols() == breaks.size()``

Raises:
    RuntimeError under the conditions specified under
    coefficient_construction_methods.)""";
        } ZeroOrderHold;
        // Symbol: drake::trajectories::PiecewisePolynomial::cols
        struct /* cols */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Returns the column count of the output matrices.

Raises:
    RuntimeError if empty().)""";
        } cols;
        // Symbol: drake::trajectories::PiecewisePolynomial::derivative
        struct /* derivative */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Returns a PiecewisePolynomial where each segment is the specified
derivative of the corresponding segment in ``this``. Any rules or
limitations of Polynomial∷derivative() also apply to this function.

Derivatives evaluated at non-differentiable points return the value at
the left hand side of the interval.

Parameter ``derivative_order``:
    The order of the derivative, namely, if ``derivative_order`` = n,
    the n'th derivative of the polynomial will be returned.

Warning:
    In the event of discontinuous derivatives evaluated at breaks, it
    is not defined which polynomial (i.e., to the left or right of the
    break) will be the one that is evaluated at the break.)""";
        } derivative;
        // Symbol: drake::trajectories::PiecewisePolynomial::empty
        struct /* empty */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Returns ``True`` if this trajectory has no breaks/samples/polynomials.)""";
        } empty;
        // Symbol: drake::trajectories::PiecewisePolynomial::getPolynomial
        struct /* getPolynomial */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Gets the Polynomial with the given matrix row and column index that
corresponds to the given segment index. Equivalent to
``getPolynomialMatrix(segment_index)(row, col)``.

Note:
    Calls PiecewiseTrajectory<T>∷segment_number_range_check() to
    validate ``segment_index``.)""";
        } getPolynomial;
        // Symbol: drake::trajectories::PiecewisePolynomial::getPolynomialMatrix
        struct /* getPolynomialMatrix */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Gets the matrix of Polynomials corresponding to the given segment
index.

Warning:
    ``segment_index`` is not checked for validity.)""";
        } getPolynomialMatrix;
        // Symbol: drake::trajectories::PiecewisePolynomial::getSegmentPolynomialDegree
        struct /* getSegmentPolynomialDegree */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Gets the degree of the Polynomial with the given matrix row and column
index that corresponds to the given segment index. Equivalent to
``getPolynomial(segment_index, row, col).GetDegree()``.)""";
        } getSegmentPolynomialDegree;
        // Symbol: drake::trajectories::PiecewisePolynomial::integral
        struct /* integral */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_was_unable_to_choose_unambiguous_names =
R"""(Returns a PiecewisePolynomial that is the indefinite integral of this
one. Any rules or limitations of Polynomial∷integral() also apply to
this function.

If ``value_at_start_time`` is given, it does the following only for
the first segment: adds that constant as the constant term
(zeroth-order coefficient) of the resulting Polynomial.)""";
        } integral;
        // Symbol: drake::trajectories::PiecewisePolynomial::isApprox
        struct /* isApprox */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Checks whether a PiecewisePolynomial is approximately equal to this
one by calling Polynomial<T>∷CoefficientsAlmostEqual() on every
element of every segment.

See also:
    Polynomial<T>∷CoefficientsAlmostEqual().)""";
        } isApprox;
        // Symbol: drake::trajectories::PiecewisePolynomial::operator*
        struct /* operator_mul */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Multiplies each Polynomial in the PolynomialMatrix of ``other`` by the
corresponding Polynomial in the PolynomialMatrix of ``this`` (i.e., a
coefficient-wise multiplication). If ``this`` corresponds to t² and
``other`` corresponds to t³, ``this *= other`` will correspond to t⁵.

Raises:
    RuntimeError if every element of ``other.get_segment_times()`` is
    not within PiecewiseTrajectory∷kEpsilonTime from
    ``this->get_segment_times()``.)""";
        } operator_mul;
        // Symbol: drake::trajectories::PiecewisePolynomial::operator*=
        struct /* operator_imul */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Multiplies each Polynomial in the PolynomialMatrix of ``other`` by the
corresponding Polynomial in the PolynomialMatrix of ``this`` (i.e., a
coefficient-wise multiplication), storing the result in ``this``. If
``this`` corresponds to t² and ``other`` corresponds to t³, ``this *=
other`` will correspond to t⁵.

Raises:
    RuntimeError if every element of ``other.get_segment_times()`` is
    not within PiecewiseTrajectory∷kEpsilonTime from
    ``this->get_segment_times()``.)""";
        } operator_imul;
        // Symbol: drake::trajectories::PiecewisePolynomial::operator+
        struct /* operator_add */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Adds each Polynomial in the PolynomialMatrix of ``other`` to the
corresponding Polynomial in the PolynomialMatrix of ``this``. If
``this`` corresponds to t² and ``other`` corresponds to t³, ``this +
other`` will correspond to t³ + t².

Raises:
    RuntimeError if every element of ``other.get_segment_times()`` is
    not within PiecewiseTrajectory∷kEpsilonTime from
    ``this->get_segment_times()``.)""";
        } operator_add;
        // Symbol: drake::trajectories::PiecewisePolynomial::operator+=
        struct /* operator_iadd */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Adds each Polynomial in the PolynomialMatrix of ``other`` to the
corresponding Polynomial in the PolynomialMatrix of ``this``, storing
the result in ``this``. If ``this`` corresponds to t² and ``other``
corresponds to t³, ``this += other`` will correspond to t³ + t².

Raises:
    RuntimeError if every element of ``other.get_segment_times()`` is
    not within PiecewiseTrajectory∷kEpsilonTime from
    ``this->get_segment_times()``.)""";
        } operator_iadd;
        // Symbol: drake::trajectories::PiecewisePolynomial::operator-
        struct /* operator_sub */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_1args =
R"""(Subtracts each Polynomial in the PolynomialMatrix of ``other`` from
the corresponding Polynomial in the PolynomialMatrix of ``this``. If
``this`` corresponds to t² and ``other`` corresponds to t³, ``this -
other`` will correspond to t² - t³.

Raises:
    RuntimeError if every element of ``other.get_segment_times()`` is
    not within PiecewiseTrajectory∷kEpsilonTime from
    ``this->get_segment_times()``.)""";
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc_0args =
R"""(Implements unary minus operator. Multiplies each Polynomial in
``this`` by -1.)""";
        } operator_sub;
        // Symbol: drake::trajectories::PiecewisePolynomial::operator-=
        struct /* operator_isub */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Subtracts each Polynomial in the PolynomialMatrix of ``other`` from
the corresponding Polynomial in the PolynomialMatrix of ``this``,
storing the result in ``this``. If ``this`` corresponds to t² and
``other`` corresponds to t³, ``this -= other`` will correspond to t² -
t³.

Raises:
    RuntimeError if every element of ``other.get_segment_times()`` is
    not within PiecewiseTrajectory∷kEpsilonTime from
    ``this->get_segment_times()``.)""";
        } operator_isub;
        // Symbol: drake::trajectories::PiecewisePolynomial::rows
        struct /* rows */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Returns the row count of the output matrices.

Raises:
    RuntimeError if empty().)""";
        } rows;
        // Symbol: drake::trajectories::PiecewisePolynomial::scalarValue
        struct /* scalarValue */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Evaluates the trajectory at the given time without returning the
entire matrix. Equivalent to value(t)(row, col).

Warning:
    See warnings in value().)""";
        } scalarValue;
        // Symbol: drake::trajectories::PiecewisePolynomial::setPolynomialMatrixBlock
        struct /* setPolynomialMatrixBlock */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Replaces the specified block of the PolynomialMatrix at the given
segment index.

Note:
    Calls PiecewiseTrajectory<T>∷segment_number_range_check() to
    validate ``segment_index``.

Warning:
    This code relies upon Eigen to verify that the replacement block
    is not too large.)""";
        } setPolynomialMatrixBlock;
        // Symbol: drake::trajectories::PiecewisePolynomial::shiftRight
        struct /* shiftRight */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Adds ``offset`` to all of the breaks. ``offset`` need not be a
non-negative number. The resulting polynomial will evaluate to
pp_after(t) = pp_before(t-offset).

As an example, `offset`=2 will result in the start_time() and
end_time() being 2 seconds later.)""";
        } shiftRight;
        // Symbol: drake::trajectories::PiecewisePolynomial::slice
        struct /* slice */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Returns the PiecewisePolynomial comprising the ``num_segments``
segments starting at the specified ``start_segment_index``.

Note:
    Calls PiecewiseTrajectory<T>∷segment_number_range_check() to
    validate ``segment_index``.)""";
        } slice;
        // Symbol: drake::trajectories::PiecewisePolynomial::value
        struct /* value */ {
          // Source: drake/common/trajectories/piecewise_polynomial.h
          const char* doc =
R"""(Evaluates the PiecewisePolynomial at the given time t.

Parameter ``t``:
    The time at which to evaluate the PiecewisePolynomial.

Returns:
    The matrix of evaluated values.

Precondition:
    If T == symbolic∷Expression, ``t.is_constant()`` must be true.

Warning:
    If t does not lie in the range that the polynomial is defined
    over, the polynomial will silently be evaluated at the closest
    point to t. For example, ``value(-1)`` will return ``value(0)``
    for a polynomial defined over [0, 1].

Warning:
    See warning in the polynomial_warning "constructor overview"
    above.)""";
        } value;
      } PiecewisePolynomial;
      // Symbol: drake::trajectories::PiecewisePose
      struct /* PiecewisePose */ {
        // Source: drake/common/trajectories/piecewise_pose.h
        const char* doc =
R"""(A wrapper class that represents a pose trajectory, whose rotation part
is a PiecewiseQuaternionSlerp and the translation part is a
PiecewisePolynomial.)""";
        // Symbol: drake::trajectories::PiecewisePose::GetAcceleration
        struct /* GetAcceleration */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc =
R"""(Returns the interpolated acceleration at ``time`` or zero if ``time``
is before this trajectory's start time or after its end time.)""";
        } GetAcceleration;
        // Symbol: drake::trajectories::PiecewisePose::GetPose
        struct /* GetPose */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc =
R"""(Returns the interpolated pose at ``time``.)""";
        } GetPose;
        // Symbol: drake::trajectories::PiecewisePose::GetVelocity
        struct /* GetVelocity */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc =
R"""(Returns the interpolated velocity at ``time`` or zero if ``time`` is
before this trajectory's start time or after its end time.)""";
        } GetVelocity;
        // Symbol: drake::trajectories::PiecewisePose::IsApprox
        struct /* IsApprox */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc =
R"""(Returns true if the position and orientation trajectories are both
within ``tol`` from the other's.)""";
        } IsApprox;
        // Symbol: drake::trajectories::PiecewisePose::MakeCubicLinearWithEndLinearVelocity
        struct /* MakeCubicLinearWithEndLinearVelocity */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc =
R"""(Constructs a PiecewisePose from given ``times`` and ``poses``. A cubic
polynomial with given end velocities is used to construct the position
part. The rotational part is represented by a piecewise quaterion
trajectory. There must be at least two elements in ``times`` and
``poses``.

Parameter ``times``:
    Breaks used to build the splines.

Parameter ``poses``:
    Knots used to build the splines.

Parameter ``start_vel``:
    Start linear velocity.

Parameter ``end_vel``:
    End linear velocity.)""";
        } MakeCubicLinearWithEndLinearVelocity;
        // Symbol: drake::trajectories::PiecewisePose::MakeLinear
        struct /* MakeLinear */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc =
R"""(Constructs a PiecewisePose from given ``times`` and ``poses``. The
positions trajectory is constructed as a first-order hold. The
orientation is constructed using the quaternion slerp. There must be
at least two elements in ``times`` and ``poses``.

Parameter ``times``:
    Breaks used to build the splines.

Parameter ``poses``:
    Knots used to build the splines.)""";
        } MakeLinear;
        // Symbol: drake::trajectories::PiecewisePose::PiecewisePose<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc_0args =
R"""(Constructs an empty piecewise pose trajectory.)""";
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc_2args =
R"""(Constructor.

Parameter ``position_trajectory``:
    Position trajectory.

Parameter ``orientation_trajectory``:
    Orientation trajectory.)""";
        } ctor;
        // Symbol: drake::trajectories::PiecewisePose::get_orientation_trajectory
        struct /* get_orientation_trajectory */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc = R"""(Returns the orientation trajectory.)""";
        } get_orientation_trajectory;
        // Symbol: drake::trajectories::PiecewisePose::get_position_trajectory
        struct /* get_position_trajectory */ {
          // Source: drake/common/trajectories/piecewise_pose.h
          const char* doc = R"""(Returns the position trajectory.)""";
        } get_position_trajectory;
      } PiecewisePose;
      // Symbol: drake::trajectories::PiecewiseQuaternionSlerp
      struct /* PiecewiseQuaternionSlerp */ {
        // Source: drake/common/trajectories/piecewise_quaternion.h
        const char* doc =
R"""(A class representing a trajectory for quaternions that are
interpolated using piecewise slerp (spherical linear interpolation).
All the orientation samples are expected to be with respect to the
same parent reference frame, i.e. q_i represents the rotation R_PBi
for the orientation of frame B at the ith sample in a fixed parent
frame P. The world frame is a common choice for the parent frame. The
angular velocity and acceleration are also relative to the parent
frame and expressed in the parent frame. Since there is a sign
ambiguity when using quaternions to represent orientation, namely q
and -q represent the same orientation, the internal quaternion
representations ensure that q_n.dot(q_{n+1}) >= 0. Another intuitive
way to think about this is that consecutive quaternions have the
shortest geodesic distance on the unit sphere. Note that the
quarternion value is in w, x, y, z order when represented as a
Vector4.)""";
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::Append
        struct /* Append */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_2args_time_quaternion =
R"""(Given a new Quaternion, this method adds one segment to the end of
``this``.)""";
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_2args_time_rotation_matrix =
R"""(Given a new RotationMatrix, this method adds one segment to the end of
``this``.)""";
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_2args_time_angle_axis =
R"""(Given a new AngleAxis, this method adds one segment to the end of
``this``.)""";
        } Append;
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::PiecewiseQuaternionSlerp<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_0args =
R"""(Builds an empty PiecewiseQuaternionSlerp.)""";
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_2args_breaks_quaternions =
R"""(Builds a PiecewiseQuaternionSlerp.

Raises:
    RuntimeError if breaks and quaternions have different length, or
    breaks have length < 2.)""";
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_2args_breaks_rotation_matrices =
R"""(Builds a PiecewiseQuaternionSlerp.

Raises:
    RuntimeError if breaks and rot_matrices have different length, or
    breaks have length < 2.)""";
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc_2args_breaks_angle_axes =
R"""(Builds a PiecewiseQuaternionSlerp.

Raises:
    RuntimeError if breaks and ang_axes have different length, or
    breaks have length < 2.)""";
        } ctor;
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::angular_acceleration
        struct /* angular_acceleration */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc =
R"""(Interpolates angular acceleration.

Parameter ``time``:
    Time for interpolation.

Returns:
    The interpolated angular acceleration at ``time``, which is always
    zero for slerp.)""";
        } angular_acceleration;
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::angular_velocity
        struct /* angular_velocity */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc =
R"""(Interpolates angular velocity.

Parameter ``time``:
    Time for interpolation.

Returns:
    The interpolated angular velocity at ``time``, which is constant
    per segment.)""";
        } angular_velocity;
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::get_quaternion_samples
        struct /* get_quaternion_samples */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc =
R"""(Getter for the internal quaternion samples.

Note:
    The returned quaternions might be different from the ones used for
    construction because the internal representations are set to
    always be the "closest" w.r.t to the previous one.

Returns:
    the internal sample points.)""";
        } get_quaternion_samples;
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::is_approx
        struct /* is_approx */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc =
R"""(Returns true if all the corresponding segment times are within ``tol``
seconds, and the angle difference between the corresponding quaternion
sample points are within ``tol`` (using ``ExtractDoubleOrThrow``).)""";
        } is_approx;
        // Symbol: drake::trajectories::PiecewiseQuaternionSlerp::orientation
        struct /* orientation */ {
          // Source: drake/common/trajectories/piecewise_quaternion.h
          const char* doc =
R"""(Interpolates orientation.

Parameter ``time``:
    Time for interpolation.

Returns:
    The interpolated quaternion at ``time``.)""";
        } orientation;
      } PiecewiseQuaternionSlerp;
      // Symbol: drake::trajectories::PiecewiseTrajectory
      struct /* PiecewiseTrajectory */ {
        // Source: drake/common/trajectories/piecewise_trajectory.h
        const char* doc =
R"""(Abstract class that implements the basic logic of maintaining
consequent segments of time (delimited by ``breaks``) to implement a
trajectory that is represented by simpler logic in each segment or
"piece".)""";
        // Symbol: drake::trajectories::PiecewiseTrajectory::PiecewiseTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc =
R"""(``breaks`` increments must be greater or equal to kEpsilonTime.)""";
        } ctor;
        // Symbol: drake::trajectories::PiecewiseTrajectory::RandomSegmentTimes
        struct /* RandomSegmentTimes */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } RandomSegmentTimes;
        // Symbol: drake::trajectories::PiecewiseTrajectory::SegmentTimesEqual
        struct /* SegmentTimesEqual */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } SegmentTimesEqual;
        // Symbol: drake::trajectories::PiecewiseTrajectory::breaks
        struct /* breaks */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } breaks;
        // Symbol: drake::trajectories::PiecewiseTrajectory::do_end_time
        struct /* do_end_time */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } do_end_time;
        // Symbol: drake::trajectories::PiecewiseTrajectory::do_start_time
        struct /* do_start_time */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } do_start_time;
        // Symbol: drake::trajectories::PiecewiseTrajectory::duration
        struct /* duration */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } duration;
        // Symbol: drake::trajectories::PiecewiseTrajectory::end_time
        struct /* end_time */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } end_time;
        // Symbol: drake::trajectories::PiecewiseTrajectory::get_mutable_breaks
        struct /* get_mutable_breaks */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } get_mutable_breaks;
        // Symbol: drake::trajectories::PiecewiseTrajectory::get_number_of_segments
        struct /* get_number_of_segments */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } get_number_of_segments;
        // Symbol: drake::trajectories::PiecewiseTrajectory::get_segment_index
        struct /* get_segment_index */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } get_segment_index;
        // Symbol: drake::trajectories::PiecewiseTrajectory::get_segment_times
        struct /* get_segment_times */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } get_segment_times;
        // Symbol: drake::trajectories::PiecewiseTrajectory::is_time_in_range
        struct /* is_time_in_range */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc =
R"""(Returns true iff ``t >= getStartTime() && t <= getEndTime()``.)""";
        } is_time_in_range;
        // Symbol: drake::trajectories::PiecewiseTrajectory::segment_number_range_check
        struct /* segment_number_range_check */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } segment_number_range_check;
        // Symbol: drake::trajectories::PiecewiseTrajectory::start_time
        struct /* start_time */ {
          // Source: drake/common/trajectories/piecewise_trajectory.h
          const char* doc = R"""()""";
        } start_time;
      } PiecewiseTrajectory;
      // Symbol: drake::trajectories::StackedTrajectory
      struct /* StackedTrajectory */ {
        // Source: drake/common/trajectories/stacked_trajectory.h
        const char* doc =
R"""(A StackedTrajectory stacks the values from one or more underlying
Trajectory objects into a single Trajectory, without changing the
``%start_time()`` or ``%end_time()``.

For sequencing trajectories in time instead, see CompositeTrajectory.

All of the underlying Trajectory objects must have the same
``%start_time()`` and ``%end_time()``.

When constructed with ``rowwise`` set to true, all of the underlying
Trajectory objects must have the same number of ``%cols()`` and the
``value()`` matrix will be the **vstack** of the the trajectories in
the order they were added.

When constructed with ``rowwise`` set to false, all of the underlying
Trajectory objects must have the same number of ``%rows()`` and the
``value()`` matrix will be the **hstack** of the the trajectories in
the order they were added.)""";
        // Symbol: drake::trajectories::StackedTrajectory::Append
        struct /* Append */ {
          // Source: drake/common/trajectories/stacked_trajectory.h
          const char* doc =
R"""(Stacks another sub-Trajectory onto this. Refer to the class overview
documentation for details.

Raises:
    RuntimeError if the matrix dimension is incompatible.)""";
        } Append;
        // Symbol: drake::trajectories::StackedTrajectory::StackedTrajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/stacked_trajectory.h
          const char* doc =
R"""(Creates an empty trajectory.

Parameter ``rowwise``:
    governs the stacking order)""";
        } ctor;
      } StackedTrajectory;
      // Symbol: drake::trajectories::Trajectory
      struct /* Trajectory */ {
        // Source: drake/common/trajectories/trajectory.h
        const char* doc =
R"""(A Trajectory represents a time-varying matrix, indexed by a single
scalar time.)""";
        // Symbol: drake::trajectories::Trajectory::Clone
        struct /* Clone */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Returns:
    A deep copy of this Trajectory.)""";
        } Clone;
        // Symbol: drake::trajectories::Trajectory::DoClone
        struct /* DoClone */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } DoClone;
        // Symbol: drake::trajectories::Trajectory::DoEvalDerivative
        struct /* DoEvalDerivative */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } DoEvalDerivative;
        // Symbol: drake::trajectories::Trajectory::DoMakeDerivative
        struct /* DoMakeDerivative */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } DoMakeDerivative;
        // Symbol: drake::trajectories::Trajectory::EvalDerivative
        struct /* EvalDerivative */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Evaluates the derivative of ``this`` at the given time ``t``. Returns
the nth derivative, where ``n`` is the value of ``derivative_order``.

Raises:
    RuntimeError if derivative_order is negative.)""";
        } EvalDerivative;
        // Symbol: drake::trajectories::Trajectory::MakeDerivative
        struct /* MakeDerivative */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Takes the derivative of this Trajectory.

Parameter ``derivative_order``:
    The number of times to take the derivative before returning.

Returns:
    The nth derivative of this object.

Raises:
    RuntimeError if derivative_order is negative.)""";
        } MakeDerivative;
        // Symbol: drake::trajectories::Trajectory::Trajectory<T>
        struct /* ctor */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } ctor;
        // Symbol: drake::trajectories::Trajectory::cols
        struct /* cols */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Returns:
    The number of columns in the matrix returned by value().)""";
        } cols;
        // Symbol: drake::trajectories::Trajectory::do_cols
        struct /* do_cols */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } do_cols;
        // Symbol: drake::trajectories::Trajectory::do_end_time
        struct /* do_end_time */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } do_end_time;
        // Symbol: drake::trajectories::Trajectory::do_has_derivative
        struct /* do_has_derivative */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } do_has_derivative;
        // Symbol: drake::trajectories::Trajectory::do_rows
        struct /* do_rows */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } do_rows;
        // Symbol: drake::trajectories::Trajectory::do_start_time
        struct /* do_start_time */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } do_start_time;
        // Symbol: drake::trajectories::Trajectory::do_value
        struct /* do_value */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } do_value;
        // Symbol: drake::trajectories::Trajectory::end_time
        struct /* end_time */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } end_time;
        // Symbol: drake::trajectories::Trajectory::has_derivative
        struct /* has_derivative */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Returns true iff the Trajectory provides and implementation for
EvalDerivative() and MakeDerivative(). The derivative need not be
continuous, but should return a result for all t for which value(t)
returns a result.)""";
        } has_derivative;
        // Symbol: drake::trajectories::Trajectory::rows
        struct /* rows */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Returns:
    The number of rows in the matrix returned by value().)""";
        } rows;
        // Symbol: drake::trajectories::Trajectory::start_time
        struct /* start_time */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc = R"""()""";
        } start_time;
        // Symbol: drake::trajectories::Trajectory::value
        struct /* value */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(Evaluates the trajectory at the given time ``t``.

Parameter ``t``:
    The time at which to evaluate the trajectory.

Returns:
    The matrix of evaluated values.)""";
        } value;
        // Symbol: drake::trajectories::Trajectory::vector_values
        struct /* vector_values */ {
          // Source: drake/common/trajectories/trajectory.h
          const char* doc =
R"""(If cols()==1, then evaluates the trajectory at each time ``t``, and
returns the results as a Matrix with the ith column corresponding to
the ith time. Otherwise, if rows()==1, then evaluates the trajectory
at each time ``t``, and returns the results as a Matrix with the ith
row corresponding to the ith time.

Raises:
    RuntimeError if both cols and rows are not equal to 1.)""";
        } vector_values;
      } Trajectory;
    } trajectories;
  } drake;
} pydrake_doc_common_trajectories;

#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
