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
# The code in this file is an extraction of the relevant parts of code that
# scipy.interpolate.interp1d would run in order to resample a
# drake.systems.primitives.VectorLog and interpolate the values for the
# purposes of animation in pydrake.systems.pyplot_visualizer.PyPlotVisualizer.
#
# This function is *NOT* intended to be used externally, it should only be used
# internally by the PyPlotVisualizer class.
import numpy as np


def _resample_log_interp1d(log, timestep):
    """
    Resample the drake systems.primitives.VectorLog evenly using a reduced
    implementation of scipy.interpolate.interp1d.  The log.sample_times() will
    be resampled evenly according to specified timestep, and the log.data()
    will be recalculated via linearly interpolating with the resampled times.

    Args:
        log: The drake systems.primitives.VectorLog to resample.
        timestep: The simulation timestep, used to resample log.sample_times()
            evenly between 0 and the last time with a distance of timestep
            between each unit.

    Returns:
        (t, x): The newly sampled (times, data) for plotting, their dimensions
            will be the same along the corresponding axes.
    """
    # We may not assume that the provided sample times are sorted in general.
    # We may assume, however, that sample_times() and data() have equitable
    # dimensions as this condition is asserted in
    # systems::primitives::VectorLog::AddData(time, sample).  The
    # sample_times() rows and data() cols are the same, and we will
    # process data() on the cols (axis=1).
    x = log.data()
    t = log.sample_times()
    axis = 1
    x_copy = np.array(x, copy=True)
    t_copy = np.array(t, copy=True)
    sort_indices = np.argsort(t_copy, kind="mergesort")
    t_copy = t_copy[sort_indices]
    x_copy = np.take(x_copy, sort_indices, axis=axis)

    # Now that we are sorted, resample the times and data.
    t_resample = np.arange(0, t_copy[-1], timestep)
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
