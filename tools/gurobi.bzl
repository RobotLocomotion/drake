# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# GUROBI_PATH should be the linux64 directory in the Gurobi 6.05 release.
# TODO(david-german-tri): Add support for OS X.
def _gurobi_impl(repository_ctx):
    gurobi_path = repository_ctx.os.environ.get(
        "GUROBI_PATH", "/MISSING_GUROBI_PATH")
    repository_ctx.symlink(gurobi_path, "gurobi-distro")

    # In the Gurobi package, libgurobi60.so is just a symlink to
    # libgurobi.so.6.0.5. However, if you use libgurobi.so.6.0.5 in srcs,
    # executables that link this library will be unable to find it at runtime
    # in the Bazel sandbox, because the NEEDED statements in the executable
    # will not square with the RPATH statements. I don't really know why this
    # happens, but I suspect it might be a Bazel bug.
    BUILD = """
    cc_library(
        name = "lib",
        srcs = ["gurobi-distro/lib/libgurobi60.so"],
        hdrs = [
            "gurobi-distro/include/gurobi_c.h",
            "gurobi-distro/include/gurobi_c++.h",
        ],
        includes = ["gurobi-distro/include"],
        linkstatic = 1,
        visibility = ["//visibility:public"],
    )
    """
    BUILD = BUILD.replace("\n    ", "\n")  # Strip leading indent from lines.
    repository_ctx.file("BUILD", content=BUILD)

gurobi_repository = repository_rule(
    local = True,
    implementation = _gurobi_impl,
)
