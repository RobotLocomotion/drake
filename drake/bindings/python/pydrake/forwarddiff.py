from __future__ import print_function, absolute_import

import numpy as np

from .autodiffutils import AutoDiffXd


def derivative(function, x):
    x_ad = AutoDiffXd(x, np.ones(1))
    y_ad = function(x_ad)
    return y_ad.derivatives()[0]


def gradient(function, x):
    x_ad = np.empty(x.shape, dtype=AutoDiffXd)
    for i in range(x.size):
        der = np.zeros(x.size)
        der[i] = 1
        x_ad.flat[i] = AutoDiffXd(x.flat[i], der)
    y_ad = function(x_ad)
    return y_ad[0].derivatives()


def jacobian(function, x):
    x_ad = np.empty(x.shape, dtype=np.object)
    for i in range(x.size):
        der = np.zeros(x.size)
        der[i] = 1
        x_ad.flat[i] = AutoDiffXd(x.flat[i], der)
    y_ad = function(x_ad)
    return np.vstack(y.derivatives() for y in y_ad)


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
