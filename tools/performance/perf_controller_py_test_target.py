""" Example of profiling subprograms, using the Python `perf` controller.

See README.md for usage.
"""

import time
import math

# Import these symbols to use the PerfController
from drake.tools.performance.perf_controller import (
    ThePerfController, scoped_perf_sampling)


# Define a bunch of busy work for the profiler to sample, with distinct call
# stacks.

def busy_work(fn):
    start = time.time()
    k = 0
    while True:
        result = fn(0.001 * k)
        k += 1
        now = time.time()
        duration = now - start
        if (duration > 1.0):
            return result


def busy_work_1():
    return busy_work(math.sin)


def busy_work_2():
    return busy_work(math.cos)


def busy_work_3():
    return busy_work(math.tan)


def busy_work_4():
    return busy_work(math.asin)


def busy_work_5():
    return busy_work(math.acos)


def main():
    # Get access to the singleton controller.
    controller = ThePerfController()

    # We can check if control is available, that is, if the communication
    # scheme provided by perf_controlled_record.py was set up.
    #
    # This check-and-fail isn't necessary in most programs, but here we want to
    # make sure controller commands are doing something.
    #
    # If control is not available, the controller commands will just be
    # no-ops. For most programs, it will be convenient to run them with
    # instrumentation in place, but with or without the perf control setup.
    if (not controller.is_control_available()):
        raise RuntimeError("perf control is unavailable!")

    # Note that sampling is initially turned off when running under
    # perf_controlled_record.py.

    busy_work_1()

    # We can turn sampling on (resume) and off (pause) manually.
    controller.resume()
    busy_work_2()
    controller.pause()

    busy_work_3()

    # We can use a scoped_perf_sampling context manager to turn sampling on for
    # the duration of a section of code.
    with scoped_perf_sampling():
        busy_work_4()

    busy_work_5()


if __name__ == '__main__':
    main()
