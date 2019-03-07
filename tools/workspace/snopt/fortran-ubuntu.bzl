# -*- python -*-

def fortran_library(
        name,
        srcs = [],
        **kwargs):
    """Compiles a Fortran library.  This library's symbols will have hidden
    visibility, becaused Drake binary release artifacts should never provide
    them as part of the public API.

    srcs: fortran source files (e.g., `*.f`).
    kwargs: are passed through to the bazel cc_library result.
    """

    # Compile *.f* files to *.pic.o files.
    compiler = "@gfortran//:compiler"
    objs = []
    for src in srcs:
        obj = src + ".pic.o"
        objs.append(obj)
        native.genrule(
            name = obj + "_genrule",
            srcs = [src],
            outs = [obj],
            tools = [compiler],
            cmd = "$(location {}) -fopenmp -fPIC -c $< -o $@".format(compiler),
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
        cmd = "$(AR) q $@ $(SRCS)",
        visibility = ["//visibility:private"],
    )

    # Wrap the *.pic.a file into a cc_library, so that we can attach
    # linkopts and deps.
    native.cc_library(
        name = name,
        srcs = [libname],
        linkopts = ["-Wl,--exclude-libs=" + libname],
        deps = ["@gfortran//:runtime"],
        **kwargs
    )
