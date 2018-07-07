# -*- python -*-

def fortran_library(
        name,
        srcs = [],
        deps = [],
        **kwargs):
    """Compiles a Fortran library.
    srcs: Fortran source files (e.g., `*.f`).
    deps: must be cc_library or fortran_library targets.
    kwargs: are passed through to the Bazel cc_library result.
    """
    objs = []
    for src in srcs:
        obj = src + ".pic.o"
        objs.append(obj)
        native.genrule(
            name = src + "_genrule",
            srcs = [src],
            tools = ["@gfortran//:compiler"],
            outs = [obj],
            cmd = "$(location @gfortran//:compiler) -fPIC -c $< -o $@",
            visibility = ["//visibility:private"],
        )
    native.cc_library(
        name = name,
        srcs = objs,
        deps = deps + [
            "@gfortran//:runtime",
        ],
        **kwargs
    )
