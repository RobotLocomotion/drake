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
    deps: are passed through to the bazel cc_library result
    kwargs: are passed through to the bazel cc_library result.
    """

    # Compile *.f* files to *.pic.o files.
    # Mark all symbols as private externs (aka hidden visibility).
    compiler = "@gfortran//:compiler"
    private_objs = []
    for src in srcs:
        public_obj = "default_" + src + ".pic.o"
        native.genrule(
            name = public_obj + "_genrule",
            srcs = [src],
            outs = [public_obj],
            tools = [compiler],
            cmd = "$(location {}) -fopenmp -fPIC -c $< -o $@".format(compiler),
            visibility = ["//visibility:private"],
        )
        private_obj = "hidden_" + src + ".pic.o"
        private_objs.append(private_obj)
        native.genrule(
            name = private_obj + "_genrule",
            srcs = [public_obj],
            outs = [private_obj],
            cmd = "/usr/bin/nmedit -s /dev/null -p $< -o $@",
            visibility = ["//visibility:private"],
        )

    # Wrap the *.pic.o files into a cc_library.
    native.cc_library(
        name = name,
        srcs = private_objs,
        deps = deps + ["@gfortran//:runtime"],
        **kwargs
    )
