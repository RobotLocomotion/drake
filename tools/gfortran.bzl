# -*- python -*-
# This is a Bazel repository_rule for libgfortran.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

def _find_and_symlink(repository_ctx, lib_name):
    gfortran = repository_ctx.which("gfortran")
    result = repository_ctx.execute([gfortran,
                                     "--print-file-name=" + lib_name])
    if result.return_code != 0:
        print(result.return_code, result.stdout, result.stderr)
        fail("gfortran.bzl: Could not --print-file-name=" + lib_name + ".  " +
             "Is gfortran installed?")

    path = result.stdout.strip()
    if path:
        repository_ctx.symlink(path, lib_name)
    else:
        fail("gfortran.bzl: Found gfortran but not " + lib_name + ". Yikes!")

def _gfortran_impl(repository_ctx):
    """Locate libgfortran.a and libquadmath.a. Wrap them in a cc_library."""
    if repository_ctx.os.name == "mac os x":
        suffix = ".dylib"
    else:
        suffix = ".so"

    libgfortran = "libgfortran{}".format(suffix)
    libquadmath = "libquadmath{}".format(suffix)

    _find_and_symlink(repository_ctx, libgfortran)
    _find_and_symlink(repository_ctx, libquadmath)

    BUILD = """
    cc_library(
        name = "lib",
        srcs = ["{}", "{}"],
        hdrs = [],
        linkopts = ["-ldl"],
        visibility = ["//visibility:public"],
    )
    """.format(libgfortran, libquadmath).replace(
        "\n    ", "\n")  # Strip leading indentation.

    repository_ctx.file("BUILD", content=BUILD)

gfortran_repository = repository_rule(
    local = True,
    implementation = _gfortran_impl,
)
