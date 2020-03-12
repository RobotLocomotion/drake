"""
Provides some general visualization tools using matplotlib.  This is not
related to `rendering`.
"""

import numpy as np
import matplotlib as mpl

from pydrake.symbolic import Evaluate, Jacobian, Polynomial
from pydrake.solvers.mathematicalprogram import MathematicalProgram, Solve


def plot_sublevelset_quadratic(ax, A, b=[0, 0], c=0, vertices=51, **kwargs):
    """
    Plots the 2D ellipse representing x'Ax + b'x + c <= 1, e.g.
    the one sub-level set of a quadratic form.

    Args:
        ax:     the matplotlib axis to receive the plot
        A:      an 2x2 PSD numpy array.
        b:      broadcastable to a 2x1 array
        c:      scalar
        vertices: number of sample points along the boundary
        kwargs: are passed to the matplotlib fill method

    Returns:
        the return values from matplotlib's fill command.
    """
    # TODO(russt): Add project_quadratic_form and slice_quadratic_form
    #  methods to drake::math and recommend users call them first to plot
    #  higher-dimensional ellipsoids.
    assert isinstance(ax, mpl.axes.Axes)
    A = np.asarray(A)
    assert A.shape == (2, 2), "A must be 2x2"

    # Note: this is closely related to
    # drake::math::DecomposePositiveQuadraticForm, but does not require the
    # function to be positive for all x.

    # f = x'Ax + b'x + c
    # dfdx = x'(A+A') + b'
    # H = .5*(A+A')   # aka, the symmetric part of A (note: x'Hx = x'Ax)
    # dfdx = 0 => xmin = -inv(A+A')*b = -.5 inv(H)*b
    H = .5*(A+A.T)
    xmin = np.linalg.solve(-2*H, np.reshape(b, (2, 1)))
    fmin = -xmin.T.dot(H).dot(xmin) + c  # since b = -2*H*xmin
    assert fmin <= 1, "The minimum value is > 1; there is no sub-level set " \
                      "to plot"

    # To plot the contour at f = (x-xmin)'H(x-xmin) + fmin = 1,
    # we make a circle of values y, such that: y'y = 1-fmin,
    th = np.linspace(0, 2*np.pi, vertices)
    Y = np.sqrt(1-fmin)*np.vstack([np.sin(th), np.cos(th)])
    # then choose L'*(x - xmin) = y, where H = LL'.
    L = np.linalg.cholesky(H)
    X = np.tile(xmin, vertices) + np.linalg.inv(np.transpose(L)).dot(Y)

    return ax.fill(X[0, :], X[1, :], **kwargs)


def plot_sublevelset_expression(ax, e, vertices=51, **kwargs):
    """
    Plots the 2D sub-level set e(x) <= 1, which must contain the origin.

    Args:
        ax:       the matplotlib axis to receive the plot
        e:        a symbolic expression in two variables
        vertices: number of sample points along the boundary
        kwargs:   are passed to the matplotlib fill method

    Returns:
        the return values from matplotlib's fill command.
    """

    x = list(e.GetVariables())
    assert len(x) == 2, "e must be an expression in two variables"

    # Handle the special case where e is a degree 2 polynomial.
    if e.is_polynomial():
        p = Polynomial(e)
        if p.TotalDegree() == 2:
            env = {a: 0 for a in x}
            c = e.Evaluate(env)
            e1 = e.Jacobian(x)
            b = Evaluate(e1, env)
            e2 = Jacobian(e1, x)
            A = 0.5*Evaluate(e2, env)
            return plot_sublevelset_quadratic(ax, A, b, c, vertices, **kwargs)

    # Find the level-set in polar coordinates, by sampling theta and
    # root-finding (on the scalar expression) to find a rplus and rminus.

    Xplus = np.empty((2, vertices))
    Xminus = np.empty((2, vertices))
    i = 0
    for theta in np.linspace(0, np.pi, vertices):
        prog = MathematicalProgram()
        r = prog.NewContinuousVariables(1, "r")[0]
        env = {x[0]: r*np.cos(theta), x[1]: r*np.sin(theta)}
        scalar = e.Substitute(env)
        b = prog.AddBoundingBoxConstraint(0, np.inf, r)
        prog.AddConstraint(scalar == 1)
        prog.AddQuadraticCost([1], [0], [r])
        prog.SetInitialGuess(r, 0.1)  # or anything non-zero.
        result = Solve(prog)
        assert result.is_success(), "Failed to find the level set"
        rplus = result.GetSolution(r)
        Xplus[0, i] = rplus*np.cos(theta)
        Xplus[1, i] = rplus*np.sin(theta)
        b.evaluator().UpdateLowerBound([-np.inf])
        b.evaluator().UpdateUpperBound([0])
        prog.SetInitialGuess(r, -0.1)  # or anything non-zero.
        result = Solve(prog)
        assert result.is_success(), "Failed to find the level set"
        rminus = result.GetSolution(r)
        Xminus[0, i] = rminus*np.cos(theta)
        Xminus[1, i] = rminus*np.sin(theta)
        i = i + 1

    return ax.fill(np.hstack((Xplus[0, :], Xminus[0, :])),
                   np.hstack((Xplus[1, :], Xminus[1, :])),
                   **kwargs)
