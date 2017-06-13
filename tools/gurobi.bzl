# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# Ubuntu only: GUROBI_PATH should be the linux64 directory in the Gurobi 7.02
# release.
def _gurobi_impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        gurobi_path = "/Library/gurobi702/mac64"
        repository_ctx.symlink(gurobi_path, "gurobi-distro")
        warning = "Gurobi 7.02 is not installed."

        repository_ctx.file("empty.cc", executable=False)
        srcs = ["empty.cc"]

        lib_path = repository_ctx.path("gurobi-distro/lib")
        linkopts = [
            "-L{}".format(lib_path),
            "-lgurobi70",
        ]
    else:
        gurobi_path = repository_ctx.os.environ.get("GUROBI_PATH", "")
        repository_ctx.symlink(
            gurobi_path or "/MISSING_GUROBI_PATH",
            "gurobi-distro")

        if not gurobi_path:
            warning_detail = "GUROBI_PATH is empty or unset"
        else:
            warning_detail = "GUROBI_PATH=%s is invalid" % gurobi_path
        warning = (
            "gurobi.bzl: The saved value of " + warning_detail + "; " +
            "export GUROBI_PATH to the correct value.")

        # In the Gurobi package, libgurobi70.so is just a symlink to
        # libgurobi.so.7.0.2. However, if you use libgurobi.so.7.0.2 in srcs,
        # executables that link this library will be unable to find it at runtime
        # in the Bazel sandbox, because the NEEDED statements in the executable
        # will not square with the RPATH statements. I don't really know why this
        # happens, but I suspect it might be a Bazel bug.
        srcs = ["gurobi-distro/lib/libgurobi70.so"]

        linkopts = ["-pthread"]

    BUILD = """
    hdrs = glob([
        "gurobi-distro/include/gurobi_c.h",
        "gurobi-distro/include/gurobi_c++.h",
    ])
    print("{warning}") if not hdrs else cc_library(
        name = "gurobi",
        srcs = {srcs},
        hdrs = hdrs,
        includes = ["gurobi-distro/include"],
        linkopts = {linkopts},
        visibility = ["//visibility:public"],
    )
    """.format(warning=warning, srcs=srcs, linkopts=linkopts)
    BUILD = BUILD.replace("\n    ", "\n")  # Strip leading indent from lines.
    repository_ctx.file("BUILD", content=BUILD, executable=False)

gurobi_repository = repository_rule(
    environ = ["GUROBI_PATH"],
    local = True,
    implementation = _gurobi_impl,
)

def gurobi_test_tags(gurobi_required=True):
    """Returns the test tags necessary for properly running Gurobi tests.

    By default, sets gurobi_required=True, which will require that the supplied
    tag filters include "gurobi".

    Gurobi checks a license file, and may need to contact a license server to
    check out a license. Therefore, tests that use Gurobi must have the tag
    "local", because they are non-hermetic. For the moment, we also require
    the tag "exclusive", to rate-limit license servers with a small number of
    licenses.
    """
    # TODO(david-german-tri): Find a better fix for the license server problem.
    nominal_tags = [
        "exclusive",
        "local",
    ]
    if gurobi_required:
        return nominal_tags + ["gurobi"]
    else:
        return nominal_tags
