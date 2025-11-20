load("//tools/skylark:drake_cc.bzl", "drake_cc_binary", "drake_cc_test")
load("//tools/skylark:py.bzl", "py_binary")
load("//tools/workspace:generate_file.bzl", "generate_file")

# This file provides build system sugar for crafting benchmarking programs.

def drake_cc_googlebench_binary(
        name,
        *,
        srcs,
        deps,
        data = None,
        add_test_rule,
        test_size = "small",
        test_timeout = None,
        test_args = None,
        test_display = False,
        test_tags = None):
    """Declares a testonly binary that uses google benchmark.  Automatically
    adds appropriate deps and ensures it either has an automated smoke test
    (via 'add_test_rule = True'), or else explicitly opts-out ('= False').
    """
    if not srcs:
        fail("Missing srcs")
    if add_test_rule == None:
        fail("Missing add_test_rule")
    new_deps = (deps or []) + ["@google_benchmark//:benchmark"]

    # We need this to be a cc_binary (not a cc_test) so that Bazel's flag
    # --trim_test_configuration will permit the py_experiment_binary target
    # to depend on it. In general, non-tests cannot depend on tests, and the
    # py_experiment_binary cannot be a test since it must not be sandboxed.
    drake_cc_binary(
        name = name,
        srcs = srcs,
        testonly = True,
        linkstatic = True,
        data = data,
        deps = new_deps,
    )

    if add_test_rule:
        # We can't simply use a sh_test wrapper around the cc_binary to run it
        # as a test, because that would exclude it from ASan, Memcheck, etc.
        # We'll need to compile it a second time from scratch, unfortunately.
        drake_cc_test(
            name = name + "_test",
            srcs = srcs,
            data = data,
            deps = new_deps,
            size = test_size,
            timeout = test_timeout,
            display = test_display,
            args = [
                # Google benchmark has a flag designed to do the minimal work
                # to confirm that the tests run: dry run. For tests, that's
                # sufficient.
                "--benchmark_dry_run",
            ] + (test_args or []),
            tags = (test_tags or []) + ["nolint", "no_kcov"],
        )

def drake_py_experiment_binary(name, *, googlebench_binary, **kwargs):
    """Declares a testonly binary that wraps google benchmark binary with
    machinery to run it under controlled conditions and summarize results.
    """
    if not googlebench_binary.startswith(":"):
        fail("googlebench_binary must be local to this package")
    dut = "drake/{}/{}".format(native.package_name(), googlebench_binary[1:])
    template = """
    import os, sys
    from python.runfiles import Create
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
        content = template.format(dut = repr(dut)),
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
            "@rules_python//python/runfiles",
        ],
    )
