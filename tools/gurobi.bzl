# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# GUROBI_PATH should be the linux64 directory in the Gurobi 6.05 release.
# TODO(david-german-tri): Add support for OS X.
def _gurobi_impl(repository_ctx):
    # TODO(jwnimmer-tri) Once bazelbuild/bazel#1595 is fixed, expose our
    # dependency on GUROBI_PATH.
    gurobi_path = repository_ctx.os.environ.get("GUROBI_PATH", "")
    repository_ctx.symlink(
        gurobi_path or "/MISSING_GUROBI_PATH",
        "gurobi-distro")

    # Set up an error message to be used by generated code.
    if not gurobi_path:
        warning_detail = "GUROBI_PATH is empty or unset"
    else:
        warning_detail = "GUROBI_PATH=%s is invalid" % gurobi_path
    warning = (
        "gurobi.bzl: The saved value of " + warning_detail + "; " +
        "export GUROBI_PATH to the correct value and then do 'bazel clean'.")

    # In the Gurobi package, libgurobi60.so is just a symlink to
    # libgurobi.so.6.0.5. However, if you use libgurobi.so.6.0.5 in srcs,
    # executables that link this library will be unable to find it at runtime
    # in the Bazel sandbox, because the NEEDED statements in the executable
    # will not square with the RPATH statements. I don't really know why this
    # happens, but I suspect it might be a Bazel bug.
    BUILD = """
    hdrs = glob([
        "gurobi-distro/include/gurobi_c.h",
        "gurobi-distro/include/gurobi_c++.h",
    ])
    print("{warning}") if not hdrs else cc_library(
        name = "lib",
        srcs = ["gurobi-distro/lib/libgurobi60.so"],
        hdrs = hdrs,
        includes = ["gurobi-distro/include"],
        linkstatic = 1,
        visibility = ["//visibility:public"],
    )
    """.format(warning=warning)
    BUILD = BUILD.replace("\n    ", "\n")  # Strip leading indent from lines.
    repository_ctx.file("BUILD", content=BUILD)

gurobi_repository = repository_rule(
    local = True,
    implementation = _gurobi_impl,
)
