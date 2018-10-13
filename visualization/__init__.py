from functools import wraps
from warnings import warn


def singleton_func(f):
    """Decorates a function that should only be called once
    (e.g. `init_visualization`).
    """
    state = dict(called=False, args=None, result=None)

    @wraps(f)
    def wrapped(*args, **kwargs):
        if state["called"]:
            if (args, kwargs) != state["args"]:
                warn(("Called a singleton function {} with different "
                      "arguments!").format(f))
            return state["result"]
        state.update(
            called=True, args=(args, kwargs),
            result=f(*args, **kwargs))
        return state["result"]

    return wrapped
