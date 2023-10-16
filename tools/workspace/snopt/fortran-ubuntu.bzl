load("@drake//tools/skylark:cc.bzl", "cc_library")
load("@gfortran//:version.bzl", COMPILER_MAJOR = "MAJOR")

def fortran_library(
        name,
        srcs = [],
        linkopts = [],
        deps = [],
        **kwargs):
    """Compiles a Fortran library.  This library's symbols will have hidden
    visibility, becaused Drake binary release artifacts should never provide
    them as part of the public API.

    srcs: fortran source files (e.g., `*.f`).
    linkopts: are passed through to the bazel cc_library result.
    deps: are passed through to the bazel cc_library result.
    kwargs: are passed through to the bazel cc_library result.
    """

    # Compile *.f* files to *.pic.o files.
    compiler = "@gfortran//:compiler"
    compiler_args = [
        "-fopenmp",
        "-fPIC",
    ]
    if COMPILER_MAJOR >= 10:
        # We need this for SNOPT 7.6 which has non-conforming code.
        compiler_args.append("-fallow-argument-mismatch")
    objs = []
    for src in srcs:
        obj = src + ".pic.o"
        objs.append(obj)
        native.genrule(
            name = obj + "_genrule",
            srcs = [src],
            outs = [obj],
            tools = [compiler],
            cmd = "$(location {}) {} -c $< -o $@".format(
                compiler,
                " ".join(compiler_args),
            ),
            visibility = ["//visibility:private"],
        )

    # Consolidate object code into an archive, so that we are able to tell the
    # linker to use hidden visibility for it.
    libname = "lib{}_{}.pic.a".format(
        native.repository_name()[1:],  # (Drop the leading @.)
        name,
    )
    native.genrule(
        name = name + "_ar_genrule",
        srcs = objs,
        outs = [libname],
        cmd = "$(AR) qcD $@ $(SRCS)",
        toolchains = ["@bazel_tools//tools/cpp:current_cc_toolchain"],
        visibility = ["//visibility:private"],
    )

    # Wrap the *.pic.a file into a cc_library, so that we can attach
    # linkopts and deps.
    cc_library(
        name = name,
        srcs = [libname],
        linkopts = linkopts + ["-Wl,--exclude-libs=" + libname],
        deps = deps + ["@gfortran//:runtime"],
        **kwargs
    )
