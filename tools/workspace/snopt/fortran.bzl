load("@drake//tools/skylark:cc.bzl", "cc_library")
load("@drake//tools/skylark:cc_hidden.bzl", "cc_wrap_static_archive_hidden")

def _fortran_object(
        *,
        src,
        obj,
        input_mods = [],
        output_mods = [],
        fopts = []):
    """Compiles one *.f file to one *.pic.o file, optionally with *.mod files
    for input and output.
    """
    compiler = "@gfortran//:compiler"
    compiler_args = ["-fPIC"] + fopts
    inputs = [src] + input_mods
    outputs = [obj] + output_mods
    cmd = [
        "$(location {})".format(compiler),
    ] + compiler_args + [
        "-c $(location {})".format(src),
        "-o $(location {})".format(obj),
    ] + [
        "-I $$(dirname $(location {}))".format(x)
        for x in input_mods
    ] + [
        "-J $$(dirname $(location {}))".format(x)
        for x in output_mods
    ]
    native.genrule(
        name = obj + "_genrule",
        srcs = inputs,
        outs = outputs,
        tools = [compiler],
        cmd = " ".join(cmd),
        visibility = ["//visibility:private"],
    )

def fortran_library(
        name,
        srcs = [],
        fopts = [],
        deps = [],
        _input_mods = [],
        _output_mods = [],
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
    objs = []
    for src in srcs:
        obj = src + ".pic.o"
        _fortran_object(
            src = src,
            obj = obj,
            fopts = fopts,
            input_mods = _input_mods,
            output_mods = _output_mods,
        )
        objs.append(obj)

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
        deps = [
            "_{}_archive_hidden".format(name),
            "@gfortran//:runtime",
        ] + deps,
        **kwargs
    )

def fortran_module(
        name,
        *,
        src,
        extra_provides = [],
        uses = [],
        fopts = [],
        deps = []):
    fortran_library(
        name = name,
        srcs = [src],
        fopts = [],
        _input_mods = [
            x + ".mod"
            for x in uses
        ],
        _output_mods = [
            name + ".mod",
        ] + [
            x + ".mod"
            for x in extra_provides
        ],
        deps = deps + [
            ":{}".format(x)
            for x in uses
        ],
    )
    for x in extra_provides:
        native.alias(name = x, actual = name)
