from functools import wraps
from warnings import warn
import weakref


def scoped_singleton_func(f):
    """Decorates a function that should only be called once
    (e.g. `init_visualization`). A weak-reference is maintained, such that the
    lifetime of the object is not affected by this decorator.
    """
    state = dict(called=False, args=None, result_ref=None)

    @wraps(f)
    def wrapped(*args, **kwargs):
        result_ref = state["result_ref"]
        is_result_valid = result_ref and result_ref() is not None
        if state["called"] and is_result_valid:
            if (args, kwargs) != state["args"]:
                warn(("Called a singleton function {} with different "
                      "arguments!").format(f))
            return result_ref()
        result = f(*args, **kwargs)
        state.update(
            called=True, args=(args, kwargs),
            result_ref=weakref.ref(result))
        return result

    return wrapped
