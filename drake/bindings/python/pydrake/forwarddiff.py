from __future__ import absolute_import, division, print_function

import numpy as np

from .autodiffutils import AutoDiffXd


def derivative(function, x):
    """
    Compute the derivative of the function evaluated at the scalar input x
    using Eigen's automatic differentiation.
    The function should be scalar-input and scalar-output.
    """
    x_ad = AutoDiffXd(x, np.ones(1))
    y_ad = function(x_ad)
    return y_ad.derivatives()[0]


def gradient(function, x):
    """
    Compute the gradient of the function evaluated at the vector input x
    using Eigen's automatic differentiation.
    The function should be vector-input and scalar-output.
    """
    x_ad = np.empty(x.shape, dtype=AutoDiffXd)
    for i in range(x.size):
        der = np.zeros(x.size)
        der[i] = 1
        x_ad.flat[i] = AutoDiffXd(x.flat[i], der)
    y_ad = function(x_ad)
    return y_ad[0].derivatives()


def jacobian(function, x):
    """
    Compute the jacobian of the function evaluated at the vector input x
    using Eigen's automatic differentiation.
    The function should be vector-input and vector-output.
    """
    x_ad = np.empty(x.shape, dtype=np.object)
    for i in range(x.size):
        der = np.zeros(x.size)
        der[i] = 1
        x_ad.flat[i] = AutoDiffXd(x.flat[i], der)
    y_ad = function(x_ad)
    return np.vstack([y.derivatives() for y in y_ad.flat]).reshape(y_ad.shape + (-1,))


# Method overloads:
# These are obviously not a complete set of mathematical functions for
# autodiff numbers. Rather, they exist just as a demonstration of *how*
# to overload individual mathematical functions to work with both
# AutoDiff and numeric inputs.

def sin(x):
    if isinstance(x, AutoDiffXd):
        return x.sin()
    else:
        return np.sin(x)


def cos(x):
    if isinstance(x, AutoDiffXd):
        return x.cos()
    else:
        return np.cos(x)
