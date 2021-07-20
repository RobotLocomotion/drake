# -*- python -*-

load("@drake//tools/skylark:drake_cc.bzl", "drake_cc_test")
load("@drake//tools/skylark:py.bzl", "py_binary")
load("@drake//tools/workspace:generate_file.bzl", "generate_file")

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

def drake_py_experiment_binary(name, *, googlebench_binary, **kwargs):
    """Declares a testonly binary that wraps google benchmark binary with
    machinery to run it under controlled conditions and summarize results.
    """
    if not googlebench_binary.startswith(":"):
        fail("googlebench_binary must be local to this package")
    dut = "drake/{}/{}".format(native.package_name(), googlebench_binary[1:])
    _TEMPLATE = """
    import os, sys
    from bazel_tools.tools.python.runfiles.runfiles import Create
    runfiles = Create()
    tool = runfiles.Rlocation("drake/tools/performance/benchmark_tool")
    dut = runfiles.Rlocation({dut})
    assert tool and dut
    env = dict(os.environ)
    env.update(runfiles.EnvVars())
    os.execve(tool, [tool, "--binary", dut] + sys.argv[1:], env=env)
    """.replace("\n    ", "\n")
    generate_file(
        name = "{}.py".format(name),
        content = _TEMPLATE.format(dut = repr(dut)),
    )
    py_binary(
        name = name,
        testonly = True,
        srcs = [":{}.py".format(name)],
        tags = ["nolint"],
        data = [
            googlebench_binary,
            "//tools/performance:benchmark_tool",
        ],
        deps = [
            "@bazel_tools//tools/python/runfiles",
        ],
    )
