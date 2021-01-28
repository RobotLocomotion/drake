import numpy as np


def _wrap_to(value, low, high):
    assert low < high
    return ((value - low) % (high - low)) + low


def deviation_from_upright_equilibrium(x):
    return np.array([
        # The upright equilibrium theta0 is pi; rotate this angle accordingly.
        _wrap_to(x[0], 0, 2 * np.pi) - np.pi,
        _wrap_to(x[1], -np.pi, np.pi),
        x[2],
        x[3],
    ])


def final_state_cost(x_tape):
    """
    Returns the L2-norm of the deviation from the upright equilibrium at the
    final state in `x_tape`.
    """
    return np.linalg.norm(deviation_from_upright_equilibrium(x_tape[:, -1]))


def ensemble_cost(x_tapes):
    """
    Returns the total cost for the trajectories in `x_tapes`.
    """
    per_trajectory_costs = [final_state_cost(x_tape) for x_tape in x_tapes]
    # Return the L1-norm of the per-trajectory-costs, scaled by the number of
    # trajectories.
    return np.linalg.norm(per_trajectory_costs, 1) / len(x_tapes)


def is_success(x_tape):
    """
    Returns true if the final state in `x_tape` is close to the upright
    equilibrium.
    """
    return final_state_cost(x_tape) < 1e-3


def success_rate(x_tapes):
    """
    Given a list of x-tapes, returns the fraction of the total for which
    `is_success()` returns True.
    """
    return np.sum([is_success(x_tape) for x_tape in x_tapes]) / len(x_tapes)
