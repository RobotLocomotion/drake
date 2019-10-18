"""
Provides some general visualization tools using matplotlib.  This is not
related to `rendering`.
"""

import numpy as np
import matplotlib as mpl

from pydrake.symbolic import Evaluate, Jacobian, Polynomial


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
    Plots the 2D sub-level set e(x) <= 1.
    This method currently only supports quadratic forms, but our intention is
    to generalize it.

    Args:
        ax:       the matplotlib axis to receive the plot
        e:        a symbolic expression in two variables
        vertices: number of sample points along the boundary
        kwargs:   are passed to the matplotlib fill method

    Returns:
        the return values from matplotlib's fill command.
    """

    # TODO(russt): implement the more general case.
    p = Polynomial(e)
    assert p.TotalDegree() == 2

    x = list(e.GetVariables())
    env = {a: 0 for a in x}
    c = e.Evaluate(env)
    e1 = e.Jacobian(x)
    b = Evaluate(e1, env)
    e2 = Jacobian(e1, x)
    A = 0.5*Evaluate(e2, env)

    return plot_sublevelset_quadratic(ax, A, b, c, vertices, **kwargs)
