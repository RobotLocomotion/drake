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
    _find_and_symlink(repository_ctx, "libgfortran.a")
    _find_and_symlink(repository_ctx, "libquadmath.a")

    BUILD = """
    cc_library(
        name = "lib",
        srcs = ["libgfortran.a", "libquadmath.a"],
        hdrs = [],
        linkopts = ["-ldl"],
        visibility = ["//visibility:public"],
    )
    """.replace("\n    ", "\n")  # Strip leading indentation.
    repository_ctx.file("BUILD", content=BUILD)

gfortran_repository = repository_rule(
    local = True,
    implementation = _gfortran_impl,
)
