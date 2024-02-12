load("@drake//tools/skylark:cc.bzl", "cc_library")
load("@drake//tools/skylark:cc_hidden.bzl", "cc_wrap_static_archive_hidden")
load("@gfortran//:version.bzl", COMPILER_MAJOR = "MAJOR")

def fortran_library(
        name,
        srcs = [],
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

    # Collate the *.pic.o files into an `*.a` archive.
    cc_library(
        name = "_{}_archive".format(name),
        srcs = objs,
        linkstatic = True,
        visibility = ["//visibility:private"],
    )

    # Convert the archive to use hidden visibility.
    cc_wrap_static_archive_hidden(
        name = "_{}_archive_hidden".format(name),
        static_archive_name = "_{}_archive".format(name),
    )

    # Provide a cc_library with the final result.
    cc_library(
        name = name,
        deps = deps + [
            "_{}_archive_hidden".format(name),
            "@gfortran//:runtime",
        ],
        **kwargs
    )
