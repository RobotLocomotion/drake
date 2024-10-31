import functools
import sys


def actual_ref_count(o):
    """Returns the actual ref count of `o`, in the caller's scope."""
    # sys.getrefcount() always artificially adds 1 to its result, owing to the
    # machinery of python calling the native-code implementation. Since we wrap
    # it here, we have to adjust the result to account for the
    # python-implemented function call. For extra fun, the ref-count cost of a
    # python function call varies with python interpreter versions, so
    # wrapped_refcount_cost() actually measures the total (native call plus
    # python call) cost.
    @functools.cache
    def wrapped_refcount_cost():
        def wrapped(o):
            return sys.getrefcount(o)
        return wrapped(object())

    return sys.getrefcount(o) - wrapped_refcount_cost()
