# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# Ubuntu only: GUROBI_PATH should be the linux64 directory in the Gurobi 6.05
# release.
def _gurobi_impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        gurobi_path = "/Library/gurobi605/mac64"
        repository_ctx.symlink(gurobi_path, "gurobi-distro")
        warning = "Gurobi 6.05 is not installed."

        repository_ctx.file("empty.cc", executable=False)
        srcs = ["empty.cc"]

        lib_path = repository_ctx.path("gurobi-distro/lib")
        linkopts = [
            "-L{}".format(lib_path),
            "-lgurobi60",
        ]
    else:
        # TODO(jwnimmer-tri) Once bazelbuild/bazel#1595 is fixed, expose our
        # dependency on GUROBI_PATH.
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
            "export GUROBI_PATH to the correct value and then do 'bazel clean'.")

        # In the Gurobi package, libgurobi60.so is just a symlink to
        # libgurobi.so.6.0.5. However, if you use libgurobi.so.6.0.5 in srcs,
        # executables that link this library will be unable to find it at runtime
        # in the Bazel sandbox, because the NEEDED statements in the executable
        # will not square with the RPATH statements. I don't really know why this
        # happens, but I suspect it might be a Bazel bug.
        srcs = ["gurobi-distro/lib/libgurobi60.so"]

        linkopts = ["-pthread"]

    BUILD = """
    hdrs = glob([
        "gurobi-distro/include/gurobi_c.h",
        "gurobi-distro/include/gurobi_c++.h",
    ])
    print("{warning}") if not hdrs else cc_library(
        name = "lib",
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
    local = True,
    implementation = _gurobi_impl,
)
