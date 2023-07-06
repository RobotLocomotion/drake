# Copyright (c) 2001-2002 Enthought, Inc. 2003-2022, SciPy Developers.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###############################################################################
#
# The code in this file is an extraction of the relevant parts of code from
# scipy.interpolate.interp1d for the purposes of animation in
# pydrake.systems.pyplot_visualizer.PyPlotVisualizer.
#
# This function is *NOT* intended to be used externally, it should only be used
# internally by the PyPlotVisualizer class.
import numpy as np


def _resample_interp1d(t, x, time_step):
    """
    A linear interpolation only version of scipy.interpolate.interp1d.

    Args:
        t : (N,) array_like
            A 1-D array of real values.
        x : (...,N,...) array_like
            A N-D array of real values. The length of `y` along the
            interpolation axis must be equal to the length of `x`.
        time_step: The simulation time_step, used to resample t
            evenly between min(t) and the last time with a distance of
            time_step between each unit.

    Note:
        In order to correctly interpolate values, the time sequence described
        by ``t`` must be sorted in increasing order.  In the
        rare event that the ``t`` are **not** sorted, the returned values
        **will** be sorted.  Both ``t`` and ``x`` are
        copied and sorted first, then resampled and interpolated.

    Returns:
        (t, x): The newly sampled (times, data) for plotting, their dimensions
            will be the same along the corresponding axes (rows of t and cols
            of x are equivalent).
    """
    # We may not assume that the provided sample times are sorted in general.
    # We may assume, however, that t and x have equitable dimensions.
    # The number of t rows and x cols are the same, and we will
    # process x on the cols (axis=1).
    axis = 1
    x_copy = np.array(x, copy=True)
    t_copy = np.array(t, copy=True)
    sort_indices = np.argsort(t_copy, kind="mergesort")
    t_copy = t_copy[sort_indices]
    x_copy = np.take(x_copy, sort_indices, axis=axis)

    # Now that we are sorted, resample the times and data.
    t_resample = np.arange(t_copy[0], t_copy[-1], time_step)
    t_new_indices = np.searchsorted(t_copy, t_resample)
    t_new_indices = t_new_indices.clip(1, len(t_copy) - 1).astype(int)
    lo_indices = t_new_indices - 1
    hi_indices = t_new_indices

    # Resample times.
    t_lo = t_copy[lo_indices]
    t_hi = t_copy[hi_indices]

    # Resample data.
    x_axis = (axis % x_copy.ndim)
    xi = np.moveaxis(np.asarray(x_copy), x_axis, 0)
    xi = xi.reshape((xi.shape[0], -1))
    xi_lo = xi[lo_indices]
    xi_hi = xi[hi_indices]

    # Perform the linear interpolation.
    slope = (xi_hi - xi_lo) / (t_hi - t_lo)[:, None]
    xi_new = slope * (t_resample - t_lo)[:, None] + xi_lo
    x_extra_shape = x_copy.shape[:x_axis] + x_copy.shape[x_axis+1:]
    x_final = xi_new.reshape(t_resample.shape + x_extra_shape)
    if x_axis != 0 and t_resample.shape != ():
        nx = len(t_resample.shape)
        ny = len(x_extra_shape)
        s = (list(range(nx, nx + x_axis))
             + list(range(nx)) + list(range(nx+x_axis, nx+ny)))
        x_final = x_final.transpose(s)

    return t_resample, x_final
