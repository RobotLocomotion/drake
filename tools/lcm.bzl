# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

# In particular, this is a Skylark extension:
# https://bazel.build/versions/master/docs/skylark/concepts.html

def _lcm_srcs_to_outs(lcm_package, lcm_srcs, extension):
    """Return the list of filenames (prefixed by the package directory and
    switched to a new extension), based on the lcm_package= and lcm_srcs=
    parameters (see lcm_cc_library below for a description of what those
    parameters mean).
    """
    result = []
    for item in lcm_srcs:
        if not item.endswith(".lcm"):
            fail(item + " does not end with .lcm")
        result.append(lcm_package + "/" + item[:-len(".lcm")] + extension)
    return result

def _lcmgen_impl(ctx):
    """The implementation actions to invoke lcmgen.

    The ctx parameter comes from Skylark:
    https://bazel.build/versions/master/docs/skylark/lib/ctx.html
    """
    # We are given ctx.outputs.outs, which is the full path and file name of
    # the generated file we want to create.  However, the lcmgen tool places
    # its outputs into a subdirectory of the path we ask for, based on the LCM
    # message's package name.  To set the correct path, we need to both remove
    # the filename from outs (which we do via ".dirname"), as well as the
    # package-name-derived directory name (which we do via slicing off striplen
    # characters), including the '/' right before it (thus the "+ 1" below).
    striplen = len(ctx.attr.lcm_package) + 1
    for lcm_src, output in zip(ctx.files.lcm_srcs, ctx.outputs.outs):
        outpath = output.dirname[:-striplen]
        if ctx.attr.language == "cc":
            arguments = ["--cpp", "--cpp-std=c++11", "--cpp-hpath=" + outpath]
        elif ctx.attr.language == "py":
            arguments = ["--python", "--ppath=" + outpath]
        else:
            fail("Unknown language")
        ctx.action(
            inputs = [lcm_src],
            outputs = [output],
            arguments = arguments + [lcm_src.path],
            executable = ctx.executable.lcmgen,
            )
    return struct()

# Create rule to invoke lcmgen on some lcm_srcs.
# https://www.bazel.io/versions/master/docs/skylark/rules.html
_lcm_library_gen = rule(
    attrs = {
        "lcm_package": attr.string(),
        "lcm_srcs": attr.label_list(allow_files = True),
        "lcmgen": attr.label(
            cfg = "host",
            executable = True,
            default = Label("@lcm//:lcmgen"),
        ),
        "outs": attr.output_list(),
        "language": attr.string(),
    },
    output_to_genfiles = True,
    implementation = _lcmgen_impl,
)

def lcm_cc_library(
        name,
        lcm_package=None,
        lcm_srcs=None,
        includes=None,
        deps=None,
        **kwargs):
    """Declares a cc_library target based on C++ message classes generated from
    `*.lcm` files.  The lcm_srcs= list parameter specifies the `*.lcm` sources.
    The lcm_package= string parameter must match the `package ...;` statement
    in all of the `*.lcm` files in lcm_srcs.  The Bazel-standard includes= and
    deps= parameters are passed through to cc_library after adding the items
    required by the generated code.

    """
    if lcm_package == None:
        fail("lcm_package must be provided")
    if not lcm_srcs:
        fail("lcm_srcs must be provided")

    outs = _lcm_srcs_to_outs(lcm_package, lcm_srcs, ".hpp")
    _lcm_library_gen(
        name=name + "_lcm_library_gen",
        language="cc",
        lcm_package=lcm_package,
        lcm_srcs=lcm_srcs,
        outs=outs)

    newdep = "@lcm//:lcm"
    deps = list(deps or [])
    if newdep not in deps:
        deps.append(newdep)

    newinclude = "."
    includes = list(includes or [])
    if newinclude not in includes:
        includes.append(newinclude)

    native.cc_library(
        name=name,
        hdrs=outs,
        deps=deps,
        includes=includes,
        **kwargs)

def lcm_py_library(
        name,
        lcm_package=None,
        lcm_srcs=None,
        deps=None,
        imports=None,
        **kwargs):
    """Declares a py_library target based on python message classes generated
    from `*.lcm` files.  The lcm_src= list parameter specifies the `*.lcm`
    sources.  The lcm_package= string parameter must match the `package ...;`
    statement in all of the `*.lcm` files in lcm_srcs.  The Bazel-standard
    deps= and imports= parameters are passed through to py_library after
    adding the items required by the generated code.

    """
    if lcm_package == None:
        fail("lcm_package must be provided")
    if not lcm_srcs:
        fail("lcm_srcs must be provided")

    outs = _lcm_srcs_to_outs(lcm_package, lcm_srcs, ".py")
    _lcm_library_gen(
        name=name + "_lcm_library_gen",
        language="py",
        lcm_package=lcm_package,
        lcm_srcs=lcm_srcs,
        outs=outs)

    newdep = "@lcm//:lcm-python"
    deps = list(deps or [])
    if newdep not in deps:
        deps.append(newdep)

    newimport = "."
    imports = list(imports or [])
    if newimport not in imports:
        imports.append(newimport)

    native.py_library(
        name=name,
        srcs=outs,
        deps=deps,
        imports=imports,
        **kwargs)
