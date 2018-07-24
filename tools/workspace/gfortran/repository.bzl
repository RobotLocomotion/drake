# -*- python -*-

load(
    "@drake//tools/workspace:execute.bzl",
    "execute_or_fail",
    "which",
)

def _find_library(repo_ctx, compiler, lib_name):
    return execute_or_fail(
        repo_ctx,
        [compiler, "--print-file-name=" + lib_name],
    ).stdout.strip()

def _gfortran_impl(repo_ctx):
    """Locate gfortran and alias it to `:compiler`; locate libgfortran and
    libquadmath and alias then to `:runtime.`"""

    # Find the compiler.
    compiler = str(which(repo_ctx, "gfortran"))
    if not compiler:
        fail("Could not find gfortran")

    # Emit a compiler wrapper.
    repo_ctx.file("compiler.sh", content="\n".join([
        "#!/bin/sh",
        "exec {} \"$@\"".format(compiler),
    ]))

    # Find the runtime libraries based on the OS.
    if repo_ctx.os.name == "mac os x":
        suffix = ".dylib"
    else:
        suffix = ".so"
    libgfortran = "libgfortran{}".format(suffix)
    libquadmath = "libquadmath{}".format(suffix)
    libgfortran_path = _find_library(repo_ctx, compiler, libgfortran)
    libquadmath_path = _find_library(repo_ctx, compiler, libquadmath)

    # The cc_library linking is different on Ubuntu vs macOS.
    if repo_ctx.os.name == "mac os x":
        repo_ctx.file("empty.cc", executable = False)
        srcs = ["empty.cc"]
        linkopts = [
            "-L{}".format(repo_ctx.path(libgfortran_path).dirname),
            "-L{}".format(repo_ctx.path(libquadmath_path).dirname),
            "-ldl",
            "-lgfortran",
            "-lquadmath",
        ]
    else:
        repo_ctx.symlink(libgfortran_path, libgfortran)
        repo_ctx.symlink(libquadmath_path, libquadmath)
        srcs = [libgfortran, libquadmath]
        linkopts = ["-ldl"]

    # Emit the build file and return.
    BUILD = """
    sh_binary(
        name = "compiler",
        srcs = ["compiler.sh"],
        visibility = ["//visibility:public"],
    )
    cc_library(
        name = "runtime",
        srcs = {},
        linkopts = {},
        visibility = ["//visibility:public"],
    )
    """.format(srcs, linkopts).replace(
        "\n    ", "\n")  # Strip leading indentation.
    repo_ctx.file("BUILD", content = BUILD)

gfortran_repository = repository_rule(
    local = True,
    implementation = _gfortran_impl,
)
