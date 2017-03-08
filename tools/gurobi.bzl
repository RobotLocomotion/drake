# -*- python -*-
# This is a Bazel repository_rule for the Gurobi solver.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

# GUROBI_PATH should be the linux64 directory in the Gurobi 6.05 release.
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
    hdrs = [
        "gurobi-distro/include/gurobi_c.h",
        "gurobi-distro/include/gurobi_c++.h",
    ]

    if repository_ctx.os.name == "mac os x":
        install_name_tool = repository_ctx.which("install_name_tool")

        libraries = ["gurobi-distro/lib/libgurobi60.so"]

        for library in libraries:
            library_path = repository_ctx.path(library)

            result = repository_ctx.execute([
                install_name_tool,
                "-id",
                library_path,
                library_path,
            ])

            if result.return_code != 0:
                fail("Could NOT change shared library identification name",
                     attr=result.stderr)

            repository_ctx.file("empty.cc", executable=False)

            srcs = ["empty.cc"]

            bin_path = repository_ctx.path("bin")

            linkopts = [
                "-L{}".format(bin_path),
                "-lgurobi60",
            ]
    else:
        srcs = ["gurobi-distro/lib/libgurobi60.so"]

        linkopts = []

    file_content = """
cc_library(
    name = "lib",
    srcs = {},
    hdrs = {},
    includes = ["gurobi-distro/include"],
    linkopts = {},
    linkstatic = 1,
    visibility = ["//visibility:public"],
)
    """.format(srcs, hdrs, linkopts)

    repository_ctx.file("BUILD", content=file_content, executable=False)

gurobi_repository = repository_rule(
    local = True,
    implementation = _gurobi_impl,
)
