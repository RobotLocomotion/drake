# -*- python -*-

load("@drake//tools/skylark:drake_cc.bzl", "drake_cc_test")

# This file provides build system sugar for crafting benchmarking programs.

def drake_cc_googlebench_binary(
        name,
        *,
        srcs,
        deps,
        test_timeout = None,
        **kwargs):
    """Declares a testonly binary that uses google benchmark.  Automatically
    adds appropriate deps and ensures it has an automated smoke test.
    """
    if not srcs:
        fail("Missing srcs")

    # Even though this program is more like a cc_binary than a cc_test, we
    # still declare it as a test here so that when run in CI the ASan, TSan,
    # Memcheck, etc. tools will all be in full effect. (In CI, tests declared
    # as python or shell wrappers are excluded from the santizers.)
    drake_cc_test(
        name = name,
        srcs = srcs,
        args = [
            # When running as a unit test, run each function only once to save
            # time. (Once is sufficient to prove lack of runtime errors.)
            "--benchmark_min_time=0",
        ],
        timeout = test_timeout,
        linkstatic = True,
        deps = (deps or []) + [
            "@googlebenchmark//:benchmark",
        ],
        **kwargs
    )
