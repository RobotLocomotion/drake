# -*- python -*-

def fortran_library(
        name,
        srcs = [],
        deps = [],
        **kwargs):
    """Compiles a Fortran library.  All linker symbols are emitted with local
    binding, becaused Drake binary release artifacts should never provide them
    as part of the public API.

    srcs: fortran source files (e.g., `*.f`).
    deps: must be cc_library or fortran_library targets.
    kwargs: are passed through to the bazel cc_library result.
    """
    compiler = "@gfortran//:compiler"
    localizer = "@drake//tools/cc_toolchain:obj_localize"
    objs = []
    for src in srcs:
        obj = src + ".pic.o"
        objs.append(obj)
        native.genrule(
            name = src + "_genrule",
            srcs = [src],
            tools = [
                compiler,
                localizer,
            ],
            outs = [obj],
            cmd = " && ".join([
                "$(location {compiler}) -fPIC -c $< -o $@",
                "$(location {localizer}) $@",
            ]).format(
                compiler = compiler,
                localizer = localizer,
            ),
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
