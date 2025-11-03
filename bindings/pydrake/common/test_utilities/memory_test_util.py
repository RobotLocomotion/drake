import functools
import sys


def actual_ref_count(o):
    """Returns the actual ref count of `o`, in the caller's scope."""

    # sys.getrefcount() artificially adds 1 to its result in python <=3.13,
    # owing to the machinery of python calling the native-code implementation.
    # Since we wrap it here, we have to adjust the result to account for the
    # python-implemented function call. For extra fun, the ref-count cost of a
    # python function call varies with python interpreter versions, so
    # wrapped_refcount_cost() actually measures the total (native call plus
    # python call) cost.
    # In python 3.14, the ref-counting semantics have been optimized so that
    # this weirdness no longer occurs.
    # TODO(tyler-yankee): Once python 3.14 is the minimum supported by Drake,
    # this entire function should be removed.
    @functools.cache
    def wrapped_refcount_cost():
        def wrapped(o):
            return sys.getrefcount(o)

        return wrapped(object()) if sys.version_info[0:2] <= (3, 13) else 0

    return sys.getrefcount(o) - wrapped_refcount_cost()
