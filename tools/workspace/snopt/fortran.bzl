# -*- python -*-

def fortran_library(
        name,
        srcs = [],
        deps = [],
        **kwargs):
    """Compiles a Fortran library.  This library's symbols will have hidden
    visibility, becaused Drake binary release artifacts should never provide
    them as part of the public API.

    srcs: fortran source files (e.g., `*.f`).
    deps: must be cc_library or fortran_library targets.
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
            tools = [compiler],
            outs = [obj],
            cmd = " && ".join([
                "$(location {compiler}) -fPIC -c $< -o $@",
            ]).format(
                compiler = compiler,
            ),
            visibility = ["//visibility:private"],
        )

    # Consolidate object code into an archive, so that we can tell the linker
    # to use hidden visibility for it.
    libname = "lib{}_{}_{}.pic.a".format(
        native.repository_name()[1:],  # (Drop the leading @.)
        native.package_name(),
        name)
    native.genrule(
        name = name + "_ar_genrule",
        srcs = objs,
        outs = [libname],
        cmd = "ar q $@ $(SRCS)",
        visibility = ["//visibility:private"],
    )

    # Wrap the *.pic.a file into a cc_library, so that we can attach
    # dependencies and linkopts; set the linkopts to hide all of our symbols.
    native.cc_library(
        name = name,
        srcs = [libname],
        linkopts = [
            "-Wl,--exclude-libs=" + libname,
        ],
        deps = deps + [
            "@gfortran//:runtime",
        ],
        **kwargs
    )
