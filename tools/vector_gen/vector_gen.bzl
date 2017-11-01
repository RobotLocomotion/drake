# -*- python -*-

load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_library",
)
load(
    "@drake//tools/skylark:pathutils.bzl",
    "basename",
    "dirname",
    "join_paths",
)

def _relative_dirname_basename(label):
    # When computing outs derived from srcs in a different package (i.e., when
    # srcs labels have a colon), we only want their package-relative stem (the
    # dirname and basename after the colon).
    if ":" in label:
        label = label.split(":")[-1]
    return dirname(label), basename(label)

def _vector_gen_outs(srcs, kind):
    """Return the list of output filenames.  The `kind` is one of "vector"
    (foo.h, foo.cc), "translator" (foo_translator.h, foo_translator.cc),
    or "lcm" (lcmt_foo_t.lcm).  For compatiblity with past practice, C++
    output will appear under a "gen" folder, but *.lcm output will not.
    """
    # Find and remove the dirname and extension shared by all srcs.
    # For srcs in the current directory, the dirname will be ".".
    subdir, _ = _relative_dirname_basename(srcs[0])
    names = []
    for item in srcs:
        item_dirname, item_basename = _relative_dirname_basename(item)
        if item_dirname != subdir:
            fail("%s subdirectory doesn't match %s" % (item, srcs[0]))
        if not item.endswith(".named_vector"):
            fail(item + " doesn't end with .named_vector")
        name = item_basename[:-len(".named_vector")]
        names.append(name)

    # Compute outs based on kind.
    if kind == "vector":
        hdrs = [
            join_paths(subdir, "gen", name + ".h")
            for name in names
        ]
        srcs = [
            join_paths(subdir, "gen", name + ".cc")
            for name in names
        ]
        return struct(hdrs = hdrs, srcs = srcs)
    elif kind == "translator":
        hdrs = [
            join_paths(subdir, "gen", name + "_translator.h")
            for name in names
        ]
        srcs = [
            join_paths(subdir, "gen", name + "_translator.cc")
            for name in names
        ]
        return struct(hdrs = hdrs, srcs = srcs)
    elif kind == "lcm":
        outs = [
            join_paths(subdir, "lcmt_" + name + "_t.lcm")
            for name in names
        ]
        return struct(outs = outs)
    else:
        fail("Unknown kind " + kind)

def _vector_gen_impl(ctx):
    """The implementation actions to invoke vector_gen."""
    ctx.action(
        inputs = ctx.files.srcs,
        outputs = ctx.outputs.outs,
        arguments = [
            "--src=%s" % src.path for src in ctx.files.srcs
        ] + [
            "--out=%s" % out.path for out in ctx.outputs.outs
        ],
        executable = ctx.executable.lcm_vector_gen,
    )
    return struct()

# Create rule to invoke lcm_vector_gen on some `*.named_vector` srcs.
_vector_gen = rule(
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "outs": attr.output_list(),
        "lcm_vector_gen": attr.label(
            cfg = "host",
            executable = True,
            default = Label("@drake//tools/vector_gen:lcm_vector_gen"),
        ),
    },
    output_to_genfiles = True,
    implementation = _vector_gen_impl,
)

def drake_cc_vector_gen_library(
        name,
        srcs = [],
        deps = [],
        **kwargs):
    """Given the *.named_vector files in `srcs`, declare a drake_cc_library
    with the given `name`, containing the generated BasicVector subclasses for
    those `srcs`.  The `deps` are passed through to the declared library.
    """
    outs = _vector_gen_outs(srcs = srcs, kind = "vector")
    _vector_gen(
        name = name + "_codegen",
        srcs = srcs,
        outs = outs.srcs + outs.hdrs,
        visibility = [])
    drake_cc_library(
        name = name,
        srcs = outs.srcs,
        hdrs = outs.hdrs,
        deps = deps + [
            "//drake/systems/framework:vector",
            "//drake/common:essential",
        ],
        **kwargs)

def drake_cc_vector_gen_translator_library(
        name,
        srcs = [],
        deps = [],
        **kwargs):
    """Given the *.named_vector files in `srcs`, declare a drake_cc_library
    with the given `name`, containing the generated LcmAndVectorBaseTranslator
    subclasses for those `srcs`.  The `deps` are passed through to the declared
    library, and must already contain (as supplied by our caller) a matching
    drake_cc_vector_gen_library(...) label whose `srcs` are a superset of ours,
    as well as a C++ library of generated LCM bindings for the LCM message(s).
    """
    outs = _vector_gen_outs(srcs = srcs, kind = "translator")
    _vector_gen(
        name = name + "_codegen",
        srcs = srcs,
        outs = outs.srcs + outs.hdrs,
        visibility = [])
    drake_cc_library(
        name = name,
        srcs = outs.srcs,
        hdrs = outs.hdrs,
        deps = deps + [
            "//drake/common:essential",
            "//drake/systems/lcm:translator",
        ],
        **kwargs)

def drake_vector_gen_lcm_sources(
        name,
        srcs = [],
        **kwargs):
    """Given the *.named_vector files in `srcs`, generate matching LCM message
    definition source files.  For a src named foo/bar.named_vector, the output
    file will be named foo/lcmt_bar_t.lcm.
    """
    outs = _vector_gen_outs(srcs = srcs, kind = "lcm")
    _vector_gen(
        name = name,
        srcs = srcs,
        outs = outs.outs,
        **kwargs)
