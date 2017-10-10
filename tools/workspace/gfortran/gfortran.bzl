# -*- python -*-
# This is a Bazel repository_rule for libgfortran.  See
# https://www.bazel.io/versions/master/docs/skylark/repository_rules.html

def _find_library(repository_ctx, lib_name):
    gfortran = repository_ctx.which("gfortran")
    result = repository_ctx.execute([gfortran,
                                     "--print-file-name=" + lib_name])
    if result.return_code != 0:
        print(result.return_code, result.stdout, result.stderr)
        fail("gfortran.bzl: Could not --print-file-name=" + lib_name + ".  " +
             "Is gfortran installed?")

    path = result.stdout.strip()
    if not path:
        fail("gfortran.bzl: Found gfortran but not " + lib_name + ". Yikes!")

    return path

def _gfortran_impl(repository_ctx):
    """Locate libgfortran.a and libquadmath.a. Wrap them in a cc_library."""
    if repository_ctx.os.name == "mac os x":
        suffix = ".dylib"
    else:
        suffix = ".so"

    libgfortran = "libgfortran{}".format(suffix)
    libquadmath = "libquadmath{}".format(suffix)

    libgfortran_path = _find_library(repository_ctx, libgfortran)
    libquadmath_path = _find_library(repository_ctx, libquadmath)

    if repository_ctx.os.name == "mac os x":
        repository_ctx.file("empty.cc", executable = False)
        srcs = ["empty.cc"]

        libgfortran_dir = repository_ctx.path(libgfortran_path).dirname
        libquadmath_dir = repository_ctx.path(libquadmath_path).dirname

        linkopts = [
            "-L{}".format(libgfortran_dir),
            "-L{}".format(libquadmath_dir),
            "-ldl",
            "-lgfortran",
            "-lquadmath",
        ]

    else:
        repository_ctx.symlink(libgfortran_path, libgfortran)
        repository_ctx.symlink(libquadmath_path, libquadmath)

        srcs = [
            libgfortran,
            libquadmath,
        ]

        linkopts = ["-ldl"]

    BUILD = """
    cc_library(
        name = "gfortran",
        srcs = {},
        hdrs = [],
        linkopts = {},
        visibility = ["//visibility:public"],
    )
    """.format(srcs, linkopts).replace(
        "\n    ", "\n")  # Strip leading indentation.

    repository_ctx.file("BUILD", content = BUILD)

gfortran_repository = repository_rule(
    local = True,
    implementation = _gfortran_impl,
)
