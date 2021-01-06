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
load(
    "@drake//tools/skylark:python_env.bzl",
    "hermetic_python_env",
)

def _relative_dirname_basename(label):
    # When computing outs derived from srcs in a different package (i.e., when
    # srcs labels have a colon), we only want their package-relative stem (the
    # dirname and basename after the colon).
    if ":" in label:
        label = label.split(":")[-1]
    return dirname(label), basename(label)

def _vector_gen_outs(srcs, kind):
    """Return the list of output filenames.  The `kind` is either "vector"
    (foo.h, foo.cc) or "lcm" (lcmt_foo_t.lcm).  For compatibility with past
    practice, C++ output will appear under a "gen" folder, but *.lcm output
    will not.
    """

    # Find and remove the dirname and extension shared by all srcs.
    # For srcs in the current directory, the dirname will be ".".
    subdir, _ = _relative_dirname_basename(srcs[0])
    names = []
    for item in srcs:
        item_dirname, item_basename = _relative_dirname_basename(item)
        if item_dirname != subdir:
            fail("%s subdirectory doesn't match %s" % (item, srcs[0]))
        if not item.endswith("_named_vector.yaml"):
            fail(item + " doesn't match *_named_vector.yaml")
        name = item_basename[:-len("_named_vector.yaml")]
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
    ctx.actions.run(
        inputs = ctx.files.srcs,
        outputs = ctx.outputs.outs,
        arguments = [
            "--src=%s" % src.path
            for src in ctx.files.srcs
        ] + [
            "--out=%s" % out.path
            for out in ctx.outputs.outs
        ] + [
            "--include_prefix=%s" % x
            for x in [ctx.attr.include_prefix]
            if x
        ],
        env = ctx.attr.env,
        executable = ctx.executable.lcm_vector_gen,
    )
    return struct()

# Create rule to invoke lcm_vector_gen on some `*.named_vector` srcs.
_vector_gen = rule(
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "outs": attr.output_list(),
        "include_prefix": attr.string(),
        "lcm_vector_gen": attr.label(
            cfg = "host",
            executable = True,
            default = Label("@drake//tools/vector_gen:lcm_vector_gen"),
        ),
        "env": attr.string_dict(
            mandatory = True,
            allow_empty = True,
        ),
    },
    output_to_genfiles = True,
    implementation = _vector_gen_impl,
)

def cc_vector_gen(
        name,
        srcs = [],
        include_prefix = None,
        drake_workspace_name = None,
        visibility = [],
        **kwargs):
    """Given the *.named_vector files in `srcs`, declare a rule with the given
    `name` to generate C++ header(s) and source(s) of BasicVector subclasses
    for those `srcs`.  Returns a struct with fields `srcs`, `hdrs`, and `deps`
    that are appropriate for use in a cc_library rule.

    The drake_workspace_name is a required argument, and is used to formulate
    the correct `result.deps`.  When this macro is called from within Drake,
    the correct value is ""; when called from other workspaces, the correct
    value is the name of Drake's workspace, such as "@drake".

    This rule only generates C++ code -- it does not compile it; within Drake,
    use the drake_cc_vector_gen_library rule below is likely a better choice.
    It will both generate and compile the code all in one rule.  This rule is
    intended for use by external projects that do not want to use Drake's
    cc_library defaults.
    """
    if drake_workspace_name == None:
        fail("Missing required drake_workspace_name")
    outs = _vector_gen_outs(srcs = srcs, kind = "vector")
    _vector_gen(
        name = name,
        srcs = srcs,
        outs = outs.srcs + outs.hdrs,
        include_prefix = include_prefix,
        visibility = visibility,
        env = hermetic_python_env(),
        **kwargs
    )
    return struct(
        srcs = outs.srcs,
        hdrs = outs.hdrs,
        deps = [drake_workspace_name + x for x in [
            "//systems/framework:vector",
            "//common:dummy_value",
            "//common:essential",
            "//common:name_value",
            "//common:symbolic",
        ]],
    )

def drake_cc_vector_gen_library(
        name,
        srcs = [],
        deps = [],
        **kwargs):
    """Given *_named_vector.yaml files in `srcs`, declare a drake_cc_library
    with the given `name`, containing the generated BasicVector subclasses for
    those `srcs`.  The `deps` are passed through to the declared library.

    This macro is inteded for use only within Drake itself.  Other projects
    using vector_gen should copy this macro and adapt it as necessary, e.g.,
    by setting drake_workspace_name correctly.
    """
    generated = cc_vector_gen(
        name = name + "_codegen",
        srcs = srcs,
        include_prefix = "drake",
        drake_workspace_name = "",
        visibility = [],
    )
    drake_cc_library(
        name = name,
        srcs = generated.srcs,
        hdrs = generated.hdrs,
        deps = deps + generated.deps,
        **kwargs
    )

def vector_gen_lcm_sources(
        name,
        srcs = [],
        **kwargs):
    """Given *_named_vector.yaml files in `srcs`, generate matching LCM message
    definition source files.  For a src named foo/bar.named_vector, the output
    file will be named foo/lcmt_bar_t.lcm.

    This macro is not limited to use within Drake itself; other projects are
    intended to use it.
    """
    outs = _vector_gen_outs(srcs = srcs, kind = "lcm")
    _vector_gen(
        name = name,
        srcs = srcs,
        outs = outs.outs,
        env = hermetic_python_env(),
        **kwargs
    )
